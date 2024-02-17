package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
    private IntakeCommands() {}

    public static Command runIntake(
        Intake intake,
        BooleanSupplier intakeIn,
        BooleanSupplier intakeOut){
        return Commands.run(() -> {
            if(intakeIn.getAsBoolean()){
                intake.setFeederSpeed(IntakeConstants.FEEDER_SPEED);
                intake.setIndexerSpeed(IntakeConstants.INDEXER_SPEED);
            }
            else if(intakeOut.getAsBoolean()){
                intake.setFeederSpeed(-IntakeConstants.FEEDER_SPEED);
                intake.setIndexerSpeed(-IntakeConstants.INDEXER_SPEED);
            }
            else{
                intake.setFeederSpeed(0);
                intake.setIndexerSpeed(0);
            }
        }, intake);
    }
}
