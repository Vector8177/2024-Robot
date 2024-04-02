package org.vector8177.util;

import static org.vector8177.Constants.*;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.DoubleSupplier;

public class CommandXboxControllerSubsystem extends CommandXboxController implements Subsystem {
  public CommandXboxControllerSubsystem(int port) {
    super(port);
  }

  public Command rumbleCommand(DoubleSupplier left, DoubleSupplier right) {
    return this.run(
            () -> {
              super.getHID().setRumble(RumbleType.kLeftRumble, left.getAsDouble());
              super.getHID().setRumble(RumbleType.kRightRumble, right.getAsDouble());
            })
        .finallyDo(() -> super.getHID().setRumble(RumbleType.kBothRumble, 0));
  }

  public Command rumbleCommand(double intensity) {
    return rumbleCommand(() -> intensity, () -> intensity);
  }
}
