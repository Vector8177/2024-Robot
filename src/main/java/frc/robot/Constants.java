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

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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

  public static final double SHOOTER_HEIGHT = 0.43;
  public static final double SPEAKER_HEIGHT = 2.0;

  public static final class IntakeConstants {
    public static final int LEFT_FEEDER_MOTOR_ID = 60;
    public static final int RIGHT_FEEDER_MOTOR_ID = 61;
    public static final int INDEXER_MOTOR_ID = 62;

    public static final int maxIntakeMotorVoltage = 12;

    public static final double FEEDER_SPEED = 0.4d; // 0.4d;
    public static final double INDEXER_SPEED = 0.5d; // 0.5d;
  }

  public static final class ClimberConstants {
    public static final int RIGHT_MOTOR_ID = 50;
    public static final int LEFT_MOTOR_ID = 51;

    public static final double CLIMBER_GEAR_RATIO = 1 / 25d;

    public static final double rightClimberAbsoluteEncoderOffset = 0d;
    public static final double leftClimberAbsoluteEncoderOffset = 0d;

    public static final int maxClimberMotorVoltage = 12;

    public static final double climberKP = 1d;
    public static final double climberKI = 0d;
    public static final double climberKD = 0d;

    public static final double climberSpeed = 1d;

    public static final double climberTopLimit = 10d;
    public static final double climberBottomLimit = 0d;
  }

  public static final class HoodConstants {
    public static final int HOOD_MOTOR_ID = 40;
    public static final int maxHoodMotorVoltage = 12;

    // public static final double hoodPivotKP = 1d;
    public static final double hoodPivotKP = 10d;
    public static final double hoodPivotKI = 0d;
    public static final double hoodPivotKD = 0d;

    public static final double SHOOT_POSE = 0.05;
    public static final double AMP_POSE = 2.1;

    public static final double HOOD_GEAR_RATIO = 1d / (5 * 5 * 5 * 36d / 24);
  }

  public final class SwerveConstants {
    public static final double MAX_LINEAR_VELOCITY = Units.feetToMeters(16.5);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_VELOCITY = MAX_LINEAR_VELOCITY / DRIVE_BASE_RADIUS;

    public static final ModuleLimits MODULE_LIMITS =
        new ModuleLimits(
            MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY * 5.0, Units.degreesToRadians(1080));

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
      public static final double thetaP = 4.0;
      public static final double thetaI = 0.0;
      public static final double thetaD = 6.0;
      public static final double thetaTolerance = Units.degreesToRadians(2.0);
      public static final double maxAngularVelocity = MAX_ANGULAR_VELOCITY;
      public static final double maxAngularAcceleration = 30.02;
    }

    public record ModuleLimits(
        double maxDriveVelocity, double maxDriveAcceleration, double maxSteeringVelocity) {}

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
      AUTO_ALIGN
    }
  }

  public final class ShooterConstants {
    public static final int SHOOTER_PIVOT_ID = 30;
    public static final int SHOOTER_TOP_ID = 31;
    public static final int SHOOTER_BOTTOM_ID = 32;
    public static final int SHOOTER_INDEXER_ID = 33;

    public static final int SHOOTER_IR_SENSOR_PORT = 0;

    public static final double SHOOTER_FF_V = 484d;

    public static final double SHOOTER_PIVOT_INTAKE_POSITION = 2.140d;

    public static final double SHOOTER_PIVOT_AMP_POSITION = 5.931d;

    public static final double SHOOTER_FENDER_AIM = 2.508d;

    public static final double SHOOTER_LONG_SHOT = 1.85d;

    public static final double SHOOTER_GEAR_RATIO = 1 / 1.5;
    public static final double ABSOLUTE_OFFSET = 0.802d;
    public static final double SHOOTER_PIVOT_GEAR_RATIO = 1d / (3 * 3 * 4 * 64d / 24);

    // Shooter Pivot PID
    public static final double SHOOTER_PIVOT_KP = 10d;
    // public static final double SHOOTER_PIVOT_KP = 6.4d;
    public static final double SHOOTER_PIVOT_KI = 0d;
    public static final double SHOOTER_PIVOT_KD = 0d;
    public static final double PIVOT_TOLERANCE = 0.2;

    // Shooter Top Wheel PID
    public static final double SHOOTER_TOP_KP = .0001d;
    // public static final double SHOOTER_TOP_KI = .000085d;
    public static final double SHOOTER_TOP_KI = .00;
    public static final double SHOOTER_TOP_KD = 0d;

    // Shooter Bottom Wheel PID
    public static final double SHOOTER_BOTTOM_KP = 1d;
    public static final double SHOOTER_BOTTOM_KI = 0d;
    public static final double SHOOTER_BOTTOM_KD = 0d;

    public static final double SHOOTER_SPEED_TOLERANCE = 0.5;

    // Arm Feed Forward
    public static final double SHOOTER_ARM_KS = 0d;
    public static final double SHOOTER_ARM_KG = 0d;
    public static final double SHOOTER_ARM_KV = 0d;
    public static final double SHOOTER_ARM_KA = 0d;

    public static final double SHOOTER_INDEXER_SPEED = 1d;

    public static final double SHOOTER_TARGET_SPEED = .8f;

    public static final double SHOOTER_IR_TARGET_VOLTAGE = 1.5d;

    public static final double MAX_MOTOR_VOLTAGE = 12d;
  }

  public final class VisionConstants {
    public final class PoseEstimation {
      public static final double POSE_DISTANCE_CUTOFF = 10d;
      public static final double POSE_AMBIGUITY_CUTOFF = 1d;
    }

    public final class FieldConstants {
      public static final double FIELD_LENGTH = 0d;
      public static final double FIELD_WIDTH = 0d;
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
}
