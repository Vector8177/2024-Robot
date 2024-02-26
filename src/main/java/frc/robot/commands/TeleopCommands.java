package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class TeleopCommands {
    public static boolean stopIntake = false;
    public static boolean readyToShoot = false;
    public static boolean ampMode = false;

    private TeleopCommands() {}

    public static Command runShooter(Shooter shooter){
        if(readyToShoot && ampMode){
            return runAmpSequence(shooter);
        }
        else if(readyToShoot){
            return runShootSequence(shooter);
        }
        else{
            return Commands.none();
        }
    }

    private static Command runShootSequence(Shooter shooter){
        return Commands.sequence(
            Commands.runOnce(() -> {
                stopIntake = true;
                shooter.setShooterSpeed(ShooterConstants.SHOOTER_SHOOT_WHEEL_RMP);
            }, shooter),
            Commands.waitUntil(() -> shooter.getShooterTopFixedVelocity() > ShooterConstants.SHOOTER_SHOOT_WHEEL_RMP_START),
            Commands.runOnce(() -> {
                shooter.setIndexerSpeed(-ShooterConstants.SHOOTER_INDEXER_SPEED);
            }, shooter),
            Commands.waitSeconds(2),
            Commands.runOnce(
                () -> {
                    stopIntake = false;
                    shooter.setShooterSpeed(1500);
                    shooter.setIndexerSpeed(0);
                },
                shooter),
            Commands.waitUntil(() -> shooter.getShooterTopFixedVelocity() < 1600),
            Commands.runOnce(
                () -> {
                    shooter.setIndexerSpeed(0);
                    shooter.setShooterSpeed(400);
                },
                shooter),
            Commands.waitUntil(() -> shooter.getShooterTopFixedVelocity() < 500),
            Commands.runOnce(
                () -> {
                    shooter.setIndexerSpeed(0);
                    shooter.setShooterSpeed(0);
                },
                shooter)
        );
    }

    private static Command runAmpSequence(Shooter shooter){
        return Commands.sequence(
            Commands.runOnce(() ->{
                stopIntake = true;
                shooter.setIndexerSpeed(ShooterConstants.SHOOTER_INDEXER_SPEED);
            }, shooter),
            Commands.waitSeconds(1.2),
            Commands.runOnce(() ->{
                stopIntake = false;
                shooter.setIndexerSpeed(0);
            }, shooter)
        );
    }

    public static Command runIntake(Intake intake, Shooter shooter){
        return Commands.runEnd(
        () -> {
            if(!stopIntake){
                intake.setFeederSpeed(-IntakeConstants.FEEDER_SPEED);
                intake.setIndexerSpeed(-IntakeConstants.INDEXER_SPEED);
                shooter.setIndexerSpeed(-ShooterConstants.SHOOTER_INDEXER_SPEED);
            }
        },
        () -> {
            if(!stopIntake){
                intake.setFeederSpeed(0);
                intake.setIndexerSpeed(0);
                shooter.setIndexerSpeed(0);
            }
        },
        intake);
    }

    public static Command runOuttake(Intake intake, Shooter shooter){
        return Commands.runEnd(
        () -> {
            if(!stopIntake){
                intake.setFeederSpeed(IntakeConstants.FEEDER_SPEED);
                intake.setIndexerSpeed(IntakeConstants.INDEXER_SPEED);
                shooter.setIndexerSpeed(ShooterConstants.SHOOTER_INDEXER_SPEED);
            }
        },
        () -> {
            if(!stopIntake){
                intake.setFeederSpeed(0);
                intake.setIndexerSpeed(0);
                shooter.setIndexerSpeed(0);
            }
        },
        intake);
    }

    public static Command setShooterIntakePosition(Shooter shooter, Hood hood){
        return Commands.runOnce(() -> {
            readyToShoot = false;
            shooter.setPosition(ShooterConstants.SHOOTER_PIVOT_INTAKE_POSITION);
            hood.setHoodPosition(false);
        }, shooter, hood);
    }

    public static Command setShooterAmpPosition(Shooter shooter, Hood hood){

        return Commands.runOnce(() -> {
            ampMode = true;
            readyToShoot = true;
            shooter.setPosition(ShooterConstants.SHOOTER_PIVOT_AMP_POSITION);
            hood.setHoodPosition(true);
        }, shooter, hood);
    }

    public static Command setShooterShootPosition(Shooter shooter, Hood hood){
        return Commands.runOnce(() -> {
            ampMode = false;
            readyToShoot = true;
            shooter.setPosition(ShooterConstants.SHOOTER_LONG_SHOT);
            hood.setHoodPosition(false);
        }, shooter, hood);
    }
}
