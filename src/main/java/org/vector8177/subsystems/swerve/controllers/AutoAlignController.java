package org.vector8177.subsystems.swerve.controllers;

import static org.vector8177.Constants.SwerveConstants.AutoAlignConstants.*;

import org.vector8177.Constants;
import org.vector8177.Constants.SwerveConstants;
import org.vector8177.subsystems.swerve.Swerve;
import org.vector8177.util.LoggedTunableNumber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class AutoAlignController {
  private static LoggedTunableNumber thetakP =
      new LoggedTunableNumber(
          "AutoAlign/thetakP", Constants.SwerveConstants.AutoAlignConstants.thetaP);
  private static LoggedTunableNumber thetakD =
      new LoggedTunableNumber(
          "AutoAlign/thetakP", Constants.SwerveConstants.AutoAlignConstants.thetaD);
  private static LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber(
          "AutoAlign/thetaTolerance", Constants.SwerveConstants.AutoAlignConstants.thetaTolerance);
  private static LoggedTunableNumber maxAngularVelocity =
      new LoggedTunableNumber(
          "AutoAlign/angularVelocity",
          Constants.SwerveConstants.AutoAlignConstants.maxAngularVelocity);
  private static LoggedTunableNumber maxAngularAcceleration =
      new LoggedTunableNumber(
          "AutoAlign/maxAngularAcceleration",
          Constants.SwerveConstants.AutoAlignConstants.maxAngularAcceleration);

  private static final double CLOSE_RPM = 3000;
  private static final double FAR_RPM = 5000;

  private static final double CLOSE_DIST = 1;
  private static final double FAR_DIST = 4.5;

  private Pose2d goalPose = SPEAKER_POSE.getBluePose();

  private ProfiledPIDController thetaController;
  private ProfiledPIDController linearController;

  private final Swerve swerve;

  private BooleanSupplier isRedSupp;

  public AutoAlignController(BooleanSupplier isRedAlliance, Swerve swerve) {
    this.isRedSupp = isRedAlliance;
    this.swerve = swerve;

    thetaController =
        new ProfiledPIDController(
            thetakP.get(),
            0,
            thetakD.get(),
            new TrapezoidProfile.Constraints(
                SwerveConstants.MODULE_LIMITS.maxSteeringVelocity(),
                SwerveConstants.MODULE_LIMITS.maxSteeringAcceleration()));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(1.5));

    linearController =
        new ProfiledPIDController(
            drivekP,
            drivekI,
            drivekD,
            new TrapezoidProfile.Constraints(
                SwerveConstants.MODULE_LIMITS.maxDriveVelocity() / 8,
                SwerveConstants.MODULE_LIMITS.maxDriveAcceleration() / 8));

    Pose2d currentPose = swerve.getPose();
    ChassisSpeeds fieldVel = swerve.getSpeeds();

    Rotation2d rotationToTarget =
        goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    thetaController.reset(currentPose.getRotation().getRadians(), fieldVel.omegaRadiansPerSecond);

    thetaController.setGoal(goalPose.getRotation().getRadians());
    Logger.recordOutput("AutoAlign/goalPose", goalPose);
    Logger.recordOutput("AutoAlign/targetRotation", rotationToTarget);
  }

  public double updateDrive() {
    this.goalPose =
        isRedSupp.getAsBoolean() ? SPEAKER_POSE.getRedPose() : SPEAKER_POSE.getBluePose();
    Pose2d currentPose = swerve.getPose();
    Rotation2d rotationToTarget =
        goalPose
            .getTranslation()
            .minus(currentPose.getTranslation())
            .getAngle()
            .plus(Rotation2d.fromDegrees(180));

    Logger.recordOutput(
        "AutoAlign/Speaker/RotationTarget",
        new Pose2d(currentPose.getTranslation(), rotationToTarget));

    Pose3d speaker =
        new Pose3d(
            new Translation3d(goalPose.getX(), goalPose.getY(), Constants.SPEAKER_HEIGHT),
            new Rotation3d());

    Logger.recordOutput("AutoAlign/Speaker/TargetPose", speaker);

    Logger.recordOutput(
        "AutoAlign/Trajectory",
        speaker.toPose2d().getTranslation().minus(currentPose.getTranslation()));

    thetaController.setGoal(rotationToTarget.getRadians());

    double angularVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(), rotationToTarget.getRadians())
            + thetaController.getSetpoint().velocity;

    if (Math.abs(angularVelocity) <= MIN_THETA_CONTROL_EFFORT) {
      angularVelocity = 0;
    }

    Logger.recordOutput("AutoAlign/Speaker/Setpoint", angularVelocity);

    return angularVelocity;
  }

  public double updateAngle() {
    Pose2d currentPose = swerve.getPose();
    this.goalPose =
        isRedSupp.getAsBoolean() ? SPEAKER_POSE.getRedPose() : SPEAKER_POSE.getBluePose();

    Translation2d targ = goalPose.getTranslation();
    Logger.recordOutput("AutoAlign/Speaker/Trans", targ);

    Logger.recordOutput("AutoAlign/Speaker/Shooter/Current", currentPose);
    Logger.recordOutput("AutoAlign/Speaker/Shooter/Target", goalPose);
    double distToTarget = goalPose.getTranslation().getDistance(currentPose.getTranslation());
    Logger.recordOutput("AutoAlign/Speaker/ShooterPose/Distance", distToTarget);
    double returnVal =
        Rotation2d.fromDegrees(90).getRadians()
            + Math.atan((Constants.SPEAKER_HEIGHT - Constants.SHOOTER_HEIGHT) / distToTarget);
    Logger.recordOutput(
        "AutoAlign/ShooterPose/ShooterTargetPoseRaw",
        Units.radiansToDegrees(
            Math.atan((Constants.SPEAKER_HEIGHT - Constants.SHOOTER_HEIGHT) / distToTarget)));

    Logger.recordOutput(
        "AutoAlign/ShooterPose/ShooterTargetPose", Units.radiansToDegrees(returnVal));

    return returnVal;
  }

  public double getShooterTarget() {
    Pose2d currentPose = swerve.getPose();
    this.goalPose =
        isRedSupp.getAsBoolean() ? SPEAKER_POSE.getRedPose() : SPEAKER_POSE.getBluePose();

    double distance = currentPose.getTranslation().getDistance(goalPose.getTranslation());

    double target =
        CLOSE_RPM
            + (FAR_RPM - CLOSE_RPM)
                * Math.abs((FAR_DIST - CLOSE_DIST) - distance)
                / (FAR_DIST - CLOSE_DIST);

    Logger.recordOutput("AutoAlign/Shooter/TargetWheelSpeed", target);
    return target;
  }

  public double getDistanceToSpeaker() {
    Pose2d currentPose = swerve.getPose();
    this.goalPose =
        isRedSupp.getAsBoolean() ? SPEAKER_POSE.getRedPose() : SPEAKER_POSE.getBluePose();

    double distance = currentPose.getTranslation().getDistance(goalPose.getTranslation());

    return distance;
  }

  public double updateDriveAmp() {
    Pose2d currentPose = swerve.getPose();
    this.goalPose = isRedSupp.getAsBoolean() ? AMP_POSE.getRedPose() : AMP_POSE.getBluePose();
    Rotation2d rotationTarget =
        isRedSupp.getAsBoolean() ? Rotation2d.fromDegrees(90) : Rotation2d.fromDegrees(-90);
    Logger.recordOutput(
        "AutoAlign/Amp/GoalPose", new Pose2d(goalPose.getTranslation(), rotationTarget));

    double distance = currentPose.getTranslation().getDistance(goalPose.getTranslation());
    // double scalarizedLinearVelocity = linearController.calculate(distance, 0);

    // var driveVelocity =
    //     new Pose2d(
    //             new Translation2d(),
    //             currentPose.getTranslation().minus(goalPose.getTranslation()).getAngle())
    //         .transformBy(
    //             GeomUtil.toTransform2d(MathUtil.clamp(scalarizedLinearVelocity, -1, 1), 0));

    var thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), rotationTarget.getRadians());

    if (Math.abs(thetaVelocity) <= MIN_THETA_CONTROL_EFFORT) thetaVelocity = 0;

    return thetaVelocity;

    // return ChassisSpeeds.fromFieldRelativeSpeeds(
    //     driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, swerve.getRotation());
  }
}
