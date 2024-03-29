package org.vector8177.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import org.vector8177.Constants;
import org.vector8177.Constants.ClimberConstants;
import org.vector8177.Constants.HoodConstants;
import org.vector8177.Constants.IntakeActiveState;
import org.vector8177.Constants.IntakeConstants;
import org.vector8177.Constants.Mode;
import org.vector8177.Constants.ShooterConstants;
import org.vector8177.Constants.ShooterState;
import org.vector8177.subsystems.climber.Climber;
import org.vector8177.subsystems.hood.Hood;
import org.vector8177.subsystems.intake.Intake;
import org.vector8177.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class MainCommands {
  private static IntakeActiveState intakeState = IntakeActiveState.ACTIVE;

  private MainCommands() {}

  public static ShooterState getCurrentState(Shooter shooter) {
    return shooter.currentState;
  }

  public static Command runShooter(Shooter shooter, DoubleSupplier speed) {
    return new SelectCommand<>(
        Map.ofEntries(
            Map.entry(ShooterState.SHOOT, runShootSequenceNum(shooter, speed)),
            Map.entry(ShooterState.AMP, runAmpSequence(shooter))),
        () -> getCurrentState(shooter));
  }

  public static Command runShootSequence(Shooter shooter) {
    return sequence(
        runOnce(
            () -> {
              // intakeState = IntakeActiveState.STOPPED;
              shooter.setShooterSpeed(shooter.getLinearizedTargetSpeed());
            },
            shooter),
        waitUntil(
            () ->
                shooter.getShooterTopFixedVelocity()
                    > (Constants.currentMode == Mode.REAL ? shooter.getTargetSpeed() : 0)),
        runOnce(
            () -> {
              shooter.setIndexerSpeed(-ShooterConstants.SHOOTER_INDEXER_SPEED);
            },
            shooter),
        waitSeconds(1),
        runOnce(
            () -> {
              intakeState = IntakeActiveState.ACTIVE;
              shooter.setShooterSpeed(1500);
              shooter.setIndexerSpeed(0);
            },
            shooter));
  }

  public static Command runShootSequenceNum(Shooter shooter, DoubleSupplier speed) {
    return sequence(
        runOnce(
            () -> {
              // intakeState = IntakeActiveState.STOPPED;
              shooter.setShooterSpeed(speed.getAsDouble());
            },
            shooter),
        waitUntil(
            () ->
                shooter.getShooterTopFixedVelocity()
                    > (Constants.currentMode == Mode.REAL ? shooter.getTargetSpeed() - 100 : 0)),
        runOnce(
            () -> {
              shooter.setIndexerSpeed(-ShooterConstants.SHOOTER_INDEXER_SPEED);
            },
            shooter),
        waitSeconds(1),
        runOnce(
            () -> {
              intakeState = IntakeActiveState.ACTIVE;
              shooter.setShooterSpeed(00);
              shooter.setIndexerSpeed(0);
            },
            shooter));
  }

  public static Command runShootSequenceAuto(Shooter shooter, DoubleSupplier speed) {
    return sequence(
        runOnce(
            () -> {
              // intakeState = IntakeActiveState.STOPPED;
              shooter.setShooterSpeed(speed.getAsDouble());
            },
            shooter),
        waitUntil(
            () ->
                shooter.getShooterBottomFixedVelocity()
                    > (Constants.currentMode == Mode.REAL ? shooter.getTargetSpeed() - 125 : 0)),
        runOnce(
            () -> {
              shooter.setIndexerSpeed(-ShooterConstants.SHOOTER_INDEXER_SPEED);
            },
            shooter),
        waitSeconds(.2),
        runOnce(
            () -> {
              intakeState = IntakeActiveState.ACTIVE;
              shooter.setIndexerSpeed(0);
            },
            shooter));
  }

  //
  public static Command runAmpSequence(Shooter shooter) {
    return sequence(
        runOnce(
            () -> {
              // intakeState = IntakeActiveState.STOPPED;
              shooter.setIndexerSpeed(ShooterConstants.SHOOTER_INDEXER_SPEED);
            },
            shooter),
        waitSeconds(1.2),
        runOnce(
            () -> {
              intakeState = IntakeActiveState.ACTIVE;
              shooter.setIndexerSpeed(0);
            }));
  }

  public static IntakeActiveState getCurrentIntakeState() {
    return intakeState;
  }

  public static Command stopIntake(Intake intake, Shooter shooter) {
    return runOnce(
        () -> {
          shooter.setIndexerSpeed(0);
          intake.setFeederSpeed(0);
          intake.setIndexerSpeed(0);
        },
        shooter,
        intake);
  }

  // public static Command runIntake(Intake intake, Shooter shooter, Hood hood) {
  // return new SelectCommand<>(
  // Map.ofEntries(
  // Map.entry(
  // IntakeActiveState.ACTIVE,
  // sequence(
  // setShooterIntakePosition(shooter, hood),
  // runOnce(
  // () -> {
  // shooter.setIndexerSpeed(-ShooterConstants.SHOOTER_INDEXER_IN_SPEED);
  // intake.setFeederSpeed(-IntakeConstants.FEEDER_SPEED);
  // intake.setFeederSpeed(-IntakeConstants.INDEXER_SPEED);
  // },
  // shooter,
  // intake),
  // waitUntil(() -> shooter.getShooterOccupied()),
  // stopIntake(intake, shooter)))),
  // () -> getCurrentIntakeState());
  // }

  public static Command runIntake(
      Intake intake, Shooter shooter, Hood hood, Consumer<Boolean> disableAA) {
    return new SelectCommand<>(
        Map.ofEntries(
            Map.entry(
                IntakeActiveState.ACTIVE,
                sequence(
                    runOnce(() -> disableAA.accept(false)),
                    setShooterIntakePosition(shooter, hood),
                    waitSeconds(.2),
                    runOnce(
                        () -> {
                          intake.setFeederSpeed(-IntakeConstants.FEEDER_SPEED);
                          intake.setIndexerSpeed(-IntakeConstants.INDEXER_SPEED);
                          shooter.setIndexerSpeed(-ShooterConstants.SHOOTER_INDEXER_IN_SPEED);
                        },
                        intake,
                        shooter),
                    waitUntil(() -> shooter.getShooterOccupied()),
                    runOnce(
                        () -> {
                          intake.setFeederSpeed(0);
                          intake.setIndexerSpeed(0);
                          shooter.setIndexerSpeed(0);
                        },
                        intake,
                        shooter)))),
        () -> getCurrentIntakeState());
  }

  public static Command runOuttake(Intake intake, Shooter shooter) {
    return new SelectCommand<>(
        Map.ofEntries(
            Map.entry(
                IntakeActiveState.ACTIVE,
                runEnd(
                    () -> {
                      intake.setFeederSpeed(IntakeConstants.FEEDER_SPEED);
                      intake.setIndexerSpeed(IntakeConstants.INDEXER_SPEED);
                      shooter.setIndexerSpeed(ShooterConstants.SHOOTER_INDEXER_SPEED);
                    },
                    () -> {
                      intake.setFeederSpeed(0);
                      intake.setIndexerSpeed(0);
                      shooter.setIndexerSpeed(0);
                    },
                    intake))),
        () -> getCurrentIntakeState());
  }

  public static Command setShooterIntakePosition(Shooter shooter, Hood hood) {

    return sequence(
        runOnce(
            () -> {
              shooter.currentState = ShooterState.EMPTY;
              shooter.setPosition(ShooterConstants.SHOOTER_PIVOT_INTAKE_POSITION);
            },
            shooter),
        runOnce(() -> hood.setHoodPosition(false), hood));
  }

  public static Command setShooterAmpPosition(Shooter shooter, Hood hood) {

    return sequence(
        runOnce(
            () -> {
              shooter.setPosition(ShooterConstants.SHOOTER_PIVOT_AMP_POSITION);
            },
            shooter),
        waitSeconds(.2),
        runOnce(
            () -> {
              hood.setHoodPosition(true);
              shooter.currentState = ShooterState.AMP;
            },
            hood));
  }

  public static Command setShooterShootPosition(Shooter shooter, Hood hood) {

    return runOnce(
        () -> {
          shooter.currentState = ShooterState.SHOOT;
          shooter.setPosition(ShooterConstants.SHOOTER_FENDER_AIM);
          hood.setHoodPosition(false);
        },
        shooter,
        hood);
  }

  public static Command setShooterHumanIntakePosition(Shooter shooter, Hood hood) {
    return runOnce(
        () -> {
          shooter.currentState = ShooterState.EMPTY;
          shooter.setPosition(ShooterConstants.SHOOTER_HUMAN_POSITION);
          hood.setHoodPosition(HoodConstants.HP_POSE);
        });
  }

  public static Command runClimber(
      Climber climber,
      BooleanSupplier leftIncrease,
      BooleanSupplier leftDecrease,
      BooleanSupplier rightIncrease,
      BooleanSupplier rightDecrease) {
    return run(
        () -> {
          double leftPoisitionVal = 0d;
          double rightPositionVal = 0d;
          if (leftIncrease.getAsBoolean()) {
            leftPoisitionVal += ClimberConstants.climberSpeed;
          }
          if (leftDecrease.getAsBoolean()) {
            leftPoisitionVal -= ClimberConstants.climberSpeed;
          }
          if (rightIncrease.getAsBoolean()) {
            rightPositionVal += ClimberConstants.climberSpeed;
          }
          if (rightDecrease.getAsBoolean()) {
            rightPositionVal -= ClimberConstants.climberSpeed;
          }
          climber.setLeftClimberPosition(leftPoisitionVal + climber.getLeftClimberPosition());
          climber.setRightClimberPosition(rightPositionVal + climber.getRightClimberPosition());
        },
        climber);
  }
}
