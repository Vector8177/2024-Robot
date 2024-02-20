package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;

public class ShooterCommands {
  private ShooterCommands() {}

  public static Command runShooter(
      Shooter shooter, BooleanSupplier intakeIn, BooleanSupplier shoot, BooleanSupplier amp) {
    return null;
  }
}
