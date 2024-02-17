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
    public static final int LEFT_FEEDER_MOTOR_ID = 1;
    public static final int RIGHT_FEEDER_MOTOR_ID = 2;
    public static final int INDEXER_MOTOR_ID = 3;

    public static final int maxIntakeMotorVoltage = 9;
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
  }

  public static final class HoodConstants {
    public static final int HOOD_MOTOR_ID = 40;
    public static final int maxHoodMotorVoltage = 12;

    public static final double hoodPivotKP = 1d;
    public static final double hoodPivotKI = 0d;
    public static final double hoodPivotKD = 0d;
  }

  public final class SwerveConstants {
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(16.5);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    // SDS Mk4I L3 Gear Ratio - 16.5ft/s
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    public static enum CanIDs {
      FL_TURN(10),
      FL_DRIVE(20),
      FR_TURN(11),
      FR_DRIVE(21),
      BL_TURN(12),
      BL_DRIVE(22),
      BR_TURN(13),
      BR_DRIVE(23),
      PIGEON(6);

      private int id;

      CanIDs(int id) {
        this.id = id;
      }

      public int getID() {
        return id;
      }
    }

    public static enum RealSwervePID {
      DRIVE_FF_S(0.1),
      DRIVE_FF_V(0.13),
      DRIVE_P(0.05),
      DRIVE_I(0.0),
      DRIVE_D(0.0),
      TURN_P(7.0),
      TURN_I(0.0),
      TURN_D(0.0);

      private double val;

      RealSwervePID(double v) {
        this.val = v;
      }

      public double getVal() {
        return this.val;
      }
    }

    public static enum SimSwervePID {
      DRIVE_FF_S(0.0),
      DRIVE_FF_V(0.13),
      DRIVE_P(0.1),
      DRIVE_I(0.0),
      DRIVE_D(0.0),
      TURN_P(10.0),
      TURN_I(0.0),
      TURN_D(0.0);

      private double val;

      SimSwervePID(double v) {
        this.val = v;
      }

      public double getVal() {
        return this.val;
      }
    }
  }

  public final class ShooterConstants {
    public static final int SHOOTER_PIVOT_ID = 30;
    public static final int SHOOTER_TOP_ID = 31;
    public static final int SHOOTER_BOTTOM_ID = 32;
    public static final int SHOOTER_INDEXER_ID = 33;

    public static final double SHOOTER_GEAR_RATIO = 1 / 1.5;
    public static final double ABSOLUTE_OFFSET = 0d;

    // Shooter Pivot PID
    public static final double SHOOTER_PIVOT_KP = 1d;
    public static final double SHOOTER_PIVOT_KI = 0d;
    public static final double SHOOTER_PIVOT_KD = 0d;
    public static final double PIVOT_TOLERANCE = 0.2;

    // Shooter Top Wheel PID
    public static final double SHOOTER_TOP_KP = 1d;
    public static final double SHOOTER_TOP_KI = 0d;
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

    public static final double MAX_MOTOR_VOLTAGE = 12d;
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
