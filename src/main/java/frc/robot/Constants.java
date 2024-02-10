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

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
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

    public static enum CanIDs
    {
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
      CanIDs(int id)
      {
        this.id = id;
      }

      public int getID() { return id; }
    }


  }

  public final class ShooterConstants {
    public static final int SHOOTER_TOP_ID = 0;
    public static final int SHOOTER_BOTTOM_ID = 0;
    public static final int SHOOTER_PIVOT_ID = 0;
    public static final int SHOOTER_INDEXER_ID = 0;

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

      PivotUnweightedPIDConstants(double d)
      {
        this.val = d;
      }

      public double getVal()
      {
        return this.val;
      }
    }

    public static enum ShooterUnweightedPIDConstants {
      P(.1),
      I(0d),
      D(0d);

      private double val;

      ShooterUnweightedPIDConstants(double d)
      {
        this.val = d;
      }

      public double getVal()
      {
        return this.val;
      }
    }

    public static final double PIVOT_TOLERANCE = 0.2;
    public static final double SHOOTER_SPEED_TOLERANCE = 0.5;

    public static final double MAX_MOTOR_VOLTAGE = 12.0;
  }
}
