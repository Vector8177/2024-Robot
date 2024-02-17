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

  public static final class IntakeConstants {
    public static final int LEFT_MOTOR_ID = 1;
    public static final int RIGHT_MOTOR_ID = 2;
    public static final int INDEXER_MOTOR_ID = 3;

    public static final int maxIntakeMotorVoltage = 9;

    public static final double intakePositionKP = 1d;
    public static final double intakePositionKI = 0d;
    public static final double intakePositionKD = 0d;
  }

  public static final class ClimberConstants {
    public static final int RIGHT_MOTOR_ID = 50;
    public static final int LEFT_MOTOR_ID = 51;

    public static final double rightClimberAbsoluteEncoderOffset = 0d;
    public static final double leftClimberAbsoluteEncoderOffset = 0d;

    public static final int maxClimberMotorVoltage = 12;

    public static final double climberKP = 1d;
    public static final double climberKI = 0d;
    public static final double climberKD = 0d;
  }

  public static final class HoodConstants {
    public static final int HOOD_MOTOR_ID = 40;
    public static final int maxHoodMotorVoltage = 12;

    public static final double hoodPivotKP = 1d;
    public static final double hoodPivotKI = 0d;
    public static final double hoodPivotKD = 0d;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class SwerveConstants {
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(6.5);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    // SDS Mk4I L3 Gear Ratio - 16.5ft/s
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

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
  }

  public final class ShooterConstants {
    public static final int SHOOTER_PIVOT_ID = 30;
    public static final int SHOOTER_TOP_ID = 31;
    public static final int SHOOTER_BOTTOM_ID = 32;
    public static final int SHOOTER_INDEXER_ID = 33;

    public static final double SHOOTER_GEAR_RATIO = 1 / 1.5;
    public static final double ABSOLUTE_OFFSET = 0d;

    public static enum PivotUnweightedPIDConstants {
      P(.1),
      I(0d),
      D(0d),
      S(0d),
      G(0d),
      V(0d),
      A(0d);

      private double val;

      PivotUnweightedPIDConstants(double d) {
        this.val = d;
      }

      public double getVal() {
        return this.val;
      }
    }

    public static enum ShooterUnweightedPIDConstants {
      P(.1),
      I(0d),
      D(0d);

      private double val;

      ShooterUnweightedPIDConstants(double d) {
        this.val = d;
      }

      public double getVal() {
        return this.val;
      }
    }

    public static final double PIVOT_TOLERANCE = 0.2;
    public static final double SHOOTER_SPEED_TOLERANCE = 0.5;

    public static final double MAX_MOTOR_VOLTAGE = 12.0;
  }
}
