package org.vector8177.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Constants.IntakeConstants;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.shooter.Shooter;
// import java.util.function.BooleanSupplier;
// import org.littletonrobotics.junction.Logger;

public class IntakeCommands {
  private IntakeCommands() {}

  // public static Command runIntake(
  //     Intake intake, Shooter shooter, BooleanSupplier intakeIn, BooleanSupplier intakeOut) {
  //   return Commands.run(
  //       () -> {
  //         if (intakeIn.getAsBoolean()) {
  //           if (shooter.getIRSensorVoltage() < ShooterConstants.SHOOTER_IR_TARGET_VOLTAGE) {
  //             Logger.recordOutput("Shooter/IR_OUTPUT", shooter.getIRSensorVoltage());
  //             intake.setFeederSpeed(-IntakeConstants.FEEDER_SPEED);
  //             intake.setIndexerSpeed(-IntakeConstants.INDEXER_SPEED);
  //             shooter.setIndexerSpeed(-ShooterConstants.SHOOTER_INDEXER_SPEED);
  //           } else {
  //             intake.stop();
  //             shooter.stop();
  //           }
  //         } else if (intakeOut.getAsBoolean()) {
  //           intake.setFeederSpeed(IntakeConstants.FEEDER_SPEED);
  //           intake.setIndexerSpeed(IntakeConstants.INDEXER_SPEED);
  //           shooter.setIndexerSpeed(ShooterConstants.SHOOTER_INDEXER_SPEED);
  //         } else {
  //           if (!shooter.getPreparingToShoot()) {
  //             intake.setFeederSpeed(0d);
  //             intake.setIndexerSpeed(0d);
  //             shooter.setIndexerSpeed(0d);
  //           }
  //         }
  //       },
  //       intake);
  // }
}