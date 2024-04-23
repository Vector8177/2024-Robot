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

package org.vector8177;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import org.photonvision.simulation.SimCameraProperties;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static final int placeHolderMotorID = Integer.MAX_VALUE;

  public static final boolean tuningMode = true;

  public static final double SHOOTER_HEIGHT = 0.4318;
  public static final double SPEAKER_HEIGHT = 2.1;

  public static final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  public static final class IntakeConstants {
    public static final int LEFT_FEEDER_MOTOR_ID = 60;
    public static final int RIGHT_FEEDER_MOTOR_ID = 61;
    public static final int INDEXER_MOTOR_ID = 62;
    // 1.85
    // 1.55 o
    public static final int maxIntakeMotorVoltage = 12;

    public static final double FEEDER_SPEED = .8d; // 0.4d;
    public static final double INDEXER_SPEED = 0.5d; // 0.5d;
  }

  public static final class ClimberConstants {
    public static final int RIGHT_MOTOR_ID = 51;
    public static final int LEFT_MOTOR_ID = 50;

    public static final double CLIMBER_GEAR_RATIO = 1 / 125d;

    public static final double rightClimberAbsoluteEncoderOffset = 0d;
    public static final double leftClimberAbsoluteEncoderOffset = 0d;

    public static final int maxClimberMotorVoltage = 12;

    public static final double climberKP = 1d;
    public static final double climberKI = 0d;
    public static final double climberKD = 0d;

    public static final double climberSpeed = 30d;

    public static final double climberTopLimit = 20000d;
    public static final double climberBottomLimit = 0.2d;
  }

  public static final class HoodConstants {
    public static final int HOOD_MOTOR_ID = 40;
    public static final int maxHoodMotorVoltage = 12;

    // public static final double hoodPivotKP = 1d;
    public static final double hoodPivotKP = 10d;
    public static final double hoodPivotKI = 0d;
    public static final double hoodPivotKD = 0d;

    public static final double SHOOT_POSE = 0.0;
    public static final double AMP_POSE = 1.921;
    public static final double HP_POSE = .383;

    public static final double HOOD_GEAR_RATIO = 1d / (5.23 * 5.23 * 5.23 * 36d / 24);
  }

  public final class SwerveConstants {
    public static final double MAX_LINEAR_VELOCITY = Units.feetToMeters(12);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(29.0);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(24.0);
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_VELOCITY = 7.328;

    public static final ModuleLimits MODULE_LIMITS =
        new ModuleLimits(MAX_LINEAR_VELOCITY, Units.inchesToMeters(18.054), 7.328, 32.484);

    // SDS Mk4I L3 Gear Ratio - 16.5ft/s
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    public static final Translation2d[] MODULE_TRANSLATIONS =
        new Translation2d[] {
          new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
          new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
          new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
          new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
        };

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(MODULE_TRANSLATIONS);

    public final class CanID {
      public static final int FL_TURN = 10;
      public static final int FL_DRIVE = 20;
      public static final int FR_TURN = 11;
      public static final int FR_DRIVE = 21;
      public static final int BL_TURN = 12;
      public static final int BL_DRIVE = 22;
      public static final int BR_TURN = 13;
      public static final int BR_DRIVE = 23;
      public static final int PIGEON = 6;
    }

    public final class RealPID {
      public static final double FF_S = 0.1d;
      public static final double FF_V = 0.13d;
      public static final double DRIVE_P = 0.05d;
      public static final double DRIVE_I = 0d;
      public static final double DRIVE_D = 0d;
      public static final double TURN_P = 7.0d;
      public static final double TURN_I = 0d;
      public static final double TURN_D = 0d;
    }

    public final class SimPID {
      public static final double FF_S = 0d;
      public static final double FF_V = 0.13d;
      public static final double DRIVE_P = 0.1d;
      public static final double DRIVE_I = 0d;
      public static final double DRIVE_D = 0d;
      public static final double TURN_P = 10.0d;
      public static final double TURN_I = 0d;
      public static final double TURN_D = 0d;
    }

    public final class AutoAlignConstants {
      public static final double thetaP = 3.5;
      public static final double thetaI = 0;
      public static final double thetaD = 0;
      public static final double thetaTolerance = Units.degreesToRadians(1);
      public static final double maxAngularVelocity = 2;
      public static final double maxAngularAcceleration = 4;

      public static final double drivekP = 3.0;
      public static final double drivekI = 0;
      public static final double drivekD = 0;

      public static final double MIN_THETA_CONTROL_EFFORT = 0.1;

      public static final AlliancePoseShifter SPEAKER_POSE =
          new AlliancePoseShifter(5.55, 16.68 - .28, -.1381 + .28);
      public static final AlliancePoseShifter AMP_POSE = new AlliancePoseShifter(7.9, 14.70, 1.81);
    }

    public record AlliancePoseShifter(double yPose, double redX, double blueX) {
      public Pose2d getRedPose() {
        return new Pose2d(redX, yPose, new Rotation2d());
      }

      public Pose2d getBluePose() {
        return new Pose2d(blueX, yPose, new Rotation2d());
      }
    }

    public record ModuleLimits(
        double maxDriveVelocity,
        double maxDriveAcceleration,
        double maxSteeringVelocity,
        double maxSteeringAcceleration) {}

    public record DriveConfig(
        double wheelRadius,
        double trackWidthX,
        double trackwidthY,
        double maxLinearVelocity,
        double maxLinearAcceleration,
        double maxAngularVelocity,
        double maxAngularAcceleration) {}

    public enum DriveMode {
      TELEOP,
      AUTO_ALIGN,
      AMP_ALIGN
    }
  }

  public final class ShooterConstants {
    public static final int SHOOTER_PIVOT_ID = 30;
    public static final int SHOOTER_TOP_ID = 31;
    public static final int SHOOTER_BOTTOM_ID = 32;
    public static final int SHOOTER_INDEXER_ID = 33;

    public static final int SHOOTER_IR_SENSOR_PORT = 0;

    public static final double SHOOTER_FF_V = 444d;

    public static final double SHOOTER_PIVOT_INTAKE_POSITION = 2.12d;

    // public static final double SHOOTER_PIVOT_AMP_POSITION = 5.931d;

    public static final double SHOOTER_PIVOT_AMP_POSITION = 6.059d;

    public static final double SHOOTER_FENDER_AIM = 2.601d;

    public static final double SHOOTER_LONG_SHOT = 1.85d;

    public static final double SHOOTER_HUMAN_POSITION = 5.634d;

    public static final int SHOOT_WHEEL_RPM = 5000;
    public static final int SHOOT_RPM_CUTOFF = 4800;

    public static final double SHOOTER_GEAR_RATIO = 1 / 1.5;
    public static final double ABSOLUTE_OFFSET = 4.723d;
    public static final double SHOOTER_PIVOT_GEAR_RATIO = 1d / (3 * 3 * 4 * 64d / 24);

    public static final double TOP_BOTTOM_DIFF = 0;

    // Shooter Pivot PID
    public static final double SHOOTER_PIVOT_KP = 10d;
    // public static final double SHOOTER_PIVOT_KP = 0d;
    public static final double SHOOTER_PIVOT_KI = 0d;
    public static final double SHOOTER_PIVOT_KD = 0d;
    public static final double PIVOT_TOLERANCE = 0.1;

    // Shooter Top Wheel PID
    public static final double SHOOTER_SPEED_KP = .0001d;
    // public static final double SHOOTER_TOP_KI = .000085d;
    public static final double SHOOTER_SPEED_KI = .00;
    public static final double SHOOTER_SPEED_KD = 0d;

    public static final double SHOOTER_TOP_SIM_KP = 5d;
    public static final double SHOOTER_SIM_FF_V = 2000;

    // Shooter Bottom Wheel PID
    // public static final double SHOOTER_BOTTOM_KP = 1d;
    // public static final double SHOOTER_BOTTOM_KI = 0d;
    // public static final double SHOOTER_BOTTOM_KD = 0d;

    public static final double SHOOTER_SPEED_TOLERANCE = 0.5;

    // Arm Feed Forward
    public static final double SHOOTER_ARM_KS = 0d;
    public static final double SHOOTER_ARM_KG = 0d;
    public static final double SHOOTER_ARM_KV = 0d;
    public static final double SHOOTER_ARM_KA = 0d;

    public static final double SHOOTER_INDEXER_SPEED = 1d;
    public static final double SHOOTER_INDEXER_IN_SPEED = .55d;

    public static final double SHOOTER_TARGET_SPEED = .8f;

    public static final double SHOOTER_IR_TARGET_VOLTAGE = 2d;
    public static final double SHOOTER_IR_TARGET_VAL = 110;

    public static final double MAX_MOTOR_VOLTAGE = 12d;
  }

  public final class VisionConstants {
    public final class PoseEstimation {
      public static final double POSE_DISTANCE_CUTOFF = 10d;
      public static final double POSE_AMBIGUITY_CUTOFF = .2d;
    }

    public final class FieldConstants {
      public static final double FIELD_LENGTH = 0d;
      public static final double FIELD_WIDTH = 0d;
    }

    public static final Transform3d frontLeftCameraPosition =
        new Transform3d(
            Units.inchesToMeters(-10.847),
            Units.inchesToMeters(-1.020),
            Units.inchesToMeters(12.418),
            new Rotation3d(0, Units.degreesToRadians(30), 0)
                .rotateBy(new Rotation3d(Units.degreesToRadians(0), 0, Units.degreesToRadians(185)))
                .rotateBy(new Rotation3d(Units.degreesToRadians(180), 0, 0)));

    public static final Transform3d frontRightCameraPosition =
        new Transform3d(
            Units.inchesToMeters(-10.847),
            Units.inchesToMeters(1.020),
            Units.inchesToMeters(12.418),
            new Rotation3d(0, Units.degreesToRadians(30), 0)
                .rotateBy(new Rotation3d(Units.degreesToRadians(0), 0, Units.degreesToRadians(175)))
                .rotateBy(new Rotation3d(Units.degreesToRadians(180), 0, 0)));

    public static final String frontLeftCameraName = "flCam";
    public static final String fronRightCameraName = "frCam";

    public static final Matrix<N3, N1> normalSingleTagStdDev =
        VecBuilder.fill(.6, .6, Double.MAX_VALUE);
    public static final Matrix<N3, N1> normalMultiTagStdDev =
        VecBuilder.fill(.3, .3, Double.MAX_VALUE);

    public static final Matrix<N3, N1> highResSingleTagStdDev =
        VecBuilder.fill(.2, .2, Double.MAX_VALUE);
    public static final Matrix<N3, N1> highResMultiTagStdDev =
        VecBuilder.fill(0.05, 0.05, Double.MAX_VALUE);

    public static final SimCameraProperties OV9281_PROP = configureNormalCamera();

    public static SimCameraProperties configureNormalCamera() {
      SimCameraProperties temp = new SimCameraProperties();
      temp.setCalibration(1280, 800, Rotation2d.fromDegrees(84.47));
      temp.setCalibError(0.25, 0.10);
      temp.setFPS(40);
      temp.setAvgLatencyMs(40);
      temp.setLatencyStdDevMs(10);

      return temp;
    }

    public static final SimCameraProperties OV2311_PROP = configureHighResCamera();

    public static SimCameraProperties configureHighResCamera() {
      SimCameraProperties temp = new SimCameraProperties();
      temp.setCalibration(1600, 1200, Rotation2d.fromDegrees(84.47));
      temp.setCalibError(0.25, 0.10);
      temp.setFPS(40);
      temp.setAvgLatencyMs(40);
      temp.setLatencyStdDevMs(10);

      return temp;
    }

    public static enum CameraResolution {
      HIGH_RES,
      NORMAL
    }
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum ShooterState {
    AMP,
    SHOOT,
    EMPTY
  }

  public static enum IntakeActiveState {
    STOPPED,
    ACTIVE
  }

  public static enum IntakeState {
    INTAKING,
    NOT_INTAKING
  }
}
