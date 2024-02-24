package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {
  private ShooterCommands() {}

  public static Command setShooterPose(
      Shooter shooter, Hood hood, double shooterPose, boolean hoodUp) {
    return Commands.runOnce(
        () -> {
          shooter.setPosition(Rotation2d.fromDegrees(shooterPose).getRadians());
          hood.setHoodPosition(hoodUp);
        },
        shooter,
        hood);
  }

  // public static Command setShooterVoltage(Shooter shooter, DoubleSupplier volts) {
  //   return Commands.run(
  //       () -> {
  //         shooter.setShooterRawVolts(volts.getAsDouble());
  //       },
  //       shooter);
  // }

  // public static Command runShooter(Shooter shooter, BooleanSupplier shoot, BooleanSupplier amp) {
  //   return Commands.runOnce(
  //       () -> {
  //         if (shoot.getAsBoolean()) {
  //           shooter.setIndexerSpeed(ShooterConstants.SHOOTER_INDEXER_SPEED);
  //           shooter.setShooterSpeed(
  //               ShooterConstants.SHOOTER_TARGET_SPEED, );
  //         } else if (amp.getAsBoolean()) {
  //           shooter.setIndexerSpeed(-ShooterConstants.SHOOTER_INDEXER_SPEED);
  //         }
  //       },
  //       shooter);
  // }
}
