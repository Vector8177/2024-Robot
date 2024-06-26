// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.vector8177.subsystems.swerve;

import static edu.wpi.first.units.Units.*;
import static org.vector8177.Constants.SwerveConstants.*;
import static org.vector8177.util.VectorUtils.*;

import org.vector8177.Constants;
import org.vector8177.Constants.Mode;
import org.vector8177.Constants.SwerveConstants;
import org.vector8177.Constants.SwerveConstants.DriveMode;
import org.vector8177.Constants.SwerveConstants.ModuleLimits;
import org.vector8177.subsystems.swerve.controllers.AutoAlignController;
import org.vector8177.subsystems.vision.AprilTagVisionIO;
import org.vector8177.subsystems.vision.AprilTagVisionIOInputsAutoLogged;
import org.vector8177.util.LocalADStarAK;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class Swerve extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final ModuleLimits limits = MODULE_LIMITS;
  private final AprilTagVisionIO aprilTagVisionIO;
  private final AprilTagVisionIOInputsAutoLogged aprilTagVisionInputs =
      new AprilTagVisionIOInputsAutoLogged();

  @AutoLogOutput public boolean useVision = false;

  private Supplier<DriveMode> currentModeSupplier;

  private AutoAlignController autoAlignController = null;

  private SwerveDriveKinematics kinematics = SwerveConstants.KINEMATICS;
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private double lastTimeStamp = 0.0;
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public Swerve(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      AprilTagVisionIO aprilTagVisionIO,
      Supplier<DriveMode> currentMode) {
    this.gyroIO = gyroIO;
    this.aprilTagVisionIO = aprilTagVisionIO;
    currentModeSupplier = currentMode;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // this.s_Vision = s_Vision;

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();
    SparkMaxOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants(3, 0),
            new PIDConstants(3, 0),
            MAX_LINEAR_VELOCITY,
            DRIVE_BASE_RADIUS,
            new ReplanningConfig(true, true)),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));

    autoAlignController =
        new AutoAlignController(
            () ->
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red,
            this);
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);

    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;

    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      // Read wheel positions and deltas from each module
      if (sampleTimestamps[i] - lastTimeStamp != 0.0) {
        double dt = sampleTimestamps[i] - lastTimeStamp;

        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
          modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];

          double velocity =
              (modulePositions[moduleIndex].distanceMeters
                      - lastModulePositions[moduleIndex].distanceMeters)
                  / dt;

          double omega =
              modulePositions[moduleIndex]
                      .angle
                      .minus(lastModulePositions[moduleIndex].angle)
                      .getRadians()
                  / dt;

          moduleDeltas[moduleIndex] =
              new SwerveModulePosition(
                  modulePositions[moduleIndex].distanceMeters
                      - lastModulePositions[moduleIndex].distanceMeters,
                  modulePositions[moduleIndex].angle);

          lastModulePositions[moduleIndex] = modulePositions[moduleIndex];

          // if (Math.abs(omega) <= limits.maxSteeringVelocity()
          // && Math.abs(velocity) <= limits.maxDriveVelocity()) {
          // moduleDeltas[moduleIndex] =
          // new SwerveModulePosition(
          // modulePositions[moduleIndex].distanceMeters
          // - lastModulePositions[moduleIndex].distanceMeters,
          // modulePositions[moduleIndex].angle);

          // lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
          // } else {
          // break;
          // }
        }
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

      Logger.recordOutput("Odometry/WheelOdo", getPose());

      // if (s_Vision != null) {
      // Pose2d currentPose = getPose();
      // List<Optional<EstimatedRobotPose>> vPoses =
      // s_Vision.getEstimatedGlobalPose(getPose());

      // for (int j = 0; j < vPoses.size(); j++) {
      // vPoses.get(j).ifPresent(this::addVisionMeasurement);
      // }
      // setPose(new Pose2d(getPose().getTranslation(), currentPose.getRotation()));
      // }
    }

    if (Constants.currentMode == Mode.SIM) return;

    aprilTagVisionIO.updatePose(getPose());
    aprilTagVisionIO.updateInputs(aprilTagVisionInputs);
    Logger.processInputs("Vision/AprilTag", aprilTagVisionInputs);

    for (int i = 0; i < aprilTagVisionInputs.timestamps.length; i++) {
      double currentTimeStamp = aprilTagVisionInputs.timestamps[i];
      Pose3d currentVisionPose = aprilTagVisionInputs.visionPoses[i];
      if (currentTimeStamp >= 1.0
          && isInBetween(currentVisionPose.getX(), 0, 16.5, false)
          && isInBetween(currentVisionPose.getY(), 0, 8.5, false)
          && isInBetween(currentVisionPose.getZ(), -.75, .75, true)
          && isInBetween(currentVisionPose.getRotation().getX(), -.2, .2, false)
          && isInBetween(currentVisionPose.getRotation().getY(), -.2, .2, false)) {
        Logger.recordOutput("Odometry/VisionPose" + i, currentVisionPose.toPose2d());
        Logger.recordOutput(
            "Odometry/AprilTagStdDevs" + i,
            Arrays.copyOfRange(aprilTagVisionInputs.visionStdDevs, 3 * i, 3 * i + 3));

        if (useVision) {
          poseEstimator.addVisionMeasurement(
              currentVisionPose.toPose2d(),
              currentTimeStamp,
              VecBuilder.fill(
                  aprilTagVisionInputs.visionStdDevs[3 * i],
                  aprilTagVisionInputs.visionStdDevs[3 * i + 1],
                  aprilTagVisionInputs.visionStdDevs[3 * i + 2]));
        } else {
          Logger.recordOutput("Drive/AprilTagPose" + i, new Pose2d());
          Logger.recordOutput("Drive/AprilTagStdDevs" + i, new double[] {0.0, 0.0, 0.0});
        }
      }
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints\
    if (currentModeSupplier.get() == DriveMode.AUTO_ALIGN) {
      double omegaVel = autoAlignController.updateDrive();
      speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omegaVel);
    }
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_VELOCITY);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "Odometry/ChassisSpeeds")
  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(EstimatedRobotPose visionPose) {
    poseEstimator.addVisionMeasurement(
        visionPose.estimatedPose.toPose2d(), visionPose.timestampSeconds);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_VELOCITY;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_VELOCITY;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return SwerveConstants.MODULE_TRANSLATIONS;
  }

  public double calculateOmegaAutoAlign() {
    return autoAlignController.updateDrive();
  }

  public double calculateAngleAutoAlign() {
    return autoAlignController.updateAngle();
  }

  public double calculateSpeedAutoAlign() {
    return autoAlignController.getShooterTarget();
  }

  public double calculateDistanceToStage() {
    return autoAlignController.getDistanceToSpeaker();
  }
}
