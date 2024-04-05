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

package org.vector8177.commands;

import org.vector8177.Robot;
import org.vector8177.subsystems.shooter.Shooter;
import org.vector8177.subsystems.swerve.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveCommands {
  private static final double DEADBAND = 0.1;

  private SwerveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Swerve swerve,
      Shooter shooter,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier slowMode) {
    return Commands.run(
        () -> {
          // Apply deadband
          //   double linearMagnitude =
          //       MathUtil.applyDeadband(
          //           Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          double allianceTrans = Robot.isRedAlliance() ? -1 : 1;
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          if (slowMode.getAsBoolean()) {
            linearMagnitude /= 2d;
          }
          Rotation2d linearDirection =
              new Rotation2d(
                  xSupplier.getAsDouble() * allianceTrans, ySupplier.getAsDouble() * allianceTrans);

          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
          // Square values
          omega = Math.copySign(omega * omega, omega);
          double omegaVelocity = omega * swerve.getMaxAngularSpeedRadPerSec();

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          swerve.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * swerve.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * swerve.getMaxLinearSpeedMetersPerSec(),
                  omegaVelocity,
                  swerve.getRotation()));
        },
        swerve);
  }
}
