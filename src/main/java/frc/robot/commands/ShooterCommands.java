package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {
  private ShooterCommands() {}

  public static Command SetSpeed(Shooter shooter, double speed) {
    return Commands.run(
        () -> {
          shooter.setTargetShooterSpeed(speed, -speed);
        },
        shooter);
  }
}
