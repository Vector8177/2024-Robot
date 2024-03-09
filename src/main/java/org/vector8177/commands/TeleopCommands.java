package org.vector8177.commands;

import org.vector8177.Constants;
import org.vector8177.Constants.ClimberConstants;
import org.vector8177.Constants.HoodConstants;
import org.vector8177.Constants.IntakeConstants;
import org.vector8177.Constants.Mode;
import org.vector8177.Constants.ShooterConstants;
import org.vector8177.subsystems.climber.Climber;
import org.vector8177.subsystems.hood.Hood;
import org.vector8177.subsystems.intake.Intake;
import org.vector8177.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;

import java.util.Map;
import java.util.function.BooleanSupplier;

public class TeleopCommands {
  public static enum ShooterState {
    AMP,
    SHOOT,
    EMPTY
  }

  public static enum IntakeState {
    STOPPED,
    ACTIVE
  }

  private static IntakeState intakeState = IntakeState.ACTIVE;
  private static ShooterState currentState = ShooterState.EMPTY;

  private TeleopCommands() {}

  public static ShooterState getCurrentState() {
    return currentState;
  }

  public static Command runShooter(Shooter shooter) {
    return new SelectCommand<>(
        Map.ofEntries(
            Map.entry(ShooterState.SHOOT, runShootSequence(shooter)),
            Map.entry(ShooterState.AMP, runAmpSequence(shooter))),
        () -> getCurrentState());
  }

  public static Command runShootSequence(Shooter shooter) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              intakeState = IntakeState.STOPPED;
              shooter.setShooterSpeed(ShooterConstants.SHOOT_WHEEL_RPM);
            },
            shooter),
        Commands.waitUntil(
            () ->
                shooter.getShooterTopFixedVelocity()
                    > (Constants.currentMode == Mode.REAL ? ShooterConstants.SHOOT_RPM_CUTOFF : 0)),
        Commands.runOnce(
            () -> {
              shooter.setIndexerSpeed(-ShooterConstants.SHOOTER_INDEXER_SPEED);
            },
            shooter),
        Commands.waitSeconds(2),
        Commands.runOnce(
            () -> {
              intakeState = IntakeState.ACTIVE;
              shooter.setShooterSpeed(1500);
              shooter.setIndexerSpeed(0);
            },
            shooter));
    // Commands.waitUntil(() -> shooter.getShooterTopFixedVelocity() < 1600),
    // Commands.runOnce(
    //     () -> {
    //       shooter.setShooterSpeed(400);
    //     },
    //     shooter),
    // Commands.waitUntil(() -> shooter.getShooterTopFixedVelocity() < 500),
    // Commands.runOnce(
    //     () -> {
    //       shooter.setShooterSpeed(0);
    //     },
    //     shooter));
    // return Commands.runOnce(
    //     () -> {
    //       shooter.setShooterSpeed(3000);
    //     },
    //     shooter);
  }

  public static Command runAmpSequence(Shooter shooter) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              intakeState = IntakeState.STOPPED;
              shooter.setIndexerSpeed(ShooterConstants.SHOOTER_INDEXER_SPEED);
            },
            shooter),
        Commands.waitSeconds(1.2),
        Commands.runOnce(
            () -> {
              intakeState = IntakeState.ACTIVE;
              shooter.setIndexerSpeed(0);
            },
            shooter));
  }

  public static IntakeState getCurrentIntakeState() {
    return intakeState;
  }

  public static Command runIntake(Intake intake, Shooter shooter) {
    return new SelectCommand<>(
        Map.ofEntries(
            Map.entry(
                IntakeState.ACTIVE,
                Commands.runEnd(
                    () -> {
                      if (shooter.getIRSensorVoltage()
                          < ShooterConstants.SHOOTER_IR_TARGET_VOLTAGE) {
                        intake.setFeederSpeed(-IntakeConstants.FEEDER_SPEED);
                        intake.setIndexerSpeed(-IntakeConstants.INDEXER_SPEED);
                        shooter.setIndexerSpeed(-ShooterConstants.SHOOTER_INDEXER_IN_SPEED);
                      }
                    },
                    () -> {
                      intake.setFeederSpeed(0);
                      intake.setIndexerSpeed(0);
                      shooter.setIndexerSpeed(0);
                    },
                    intake))),
        () -> getCurrentIntakeState());
  }

  public static Command runOuttake(Intake intake, Shooter shooter) {
    return new SelectCommand<>(
        Map.ofEntries(
            Map.entry(
                IntakeState.ACTIVE,
                Commands.runEnd(
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

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              currentState = ShooterState.EMPTY;
              shooter.setPosition(ShooterConstants.SHOOTER_PIVOT_INTAKE_POSITION);
            },
            shooter),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> hood.setHoodPosition(false), hood));
  }

  public static Command setShooterAmpPosition(Shooter shooter, Hood hood) {

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              currentState = ShooterState.AMP;
              shooter.setPosition(ShooterConstants.SHOOTER_PIVOT_AMP_POSITION);
            },
            shooter),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> hood.setHoodPosition(true), hood));
  }

  public static Command setShooterShootPosition(Shooter shooter, Hood hood) {

    return Commands.runOnce(
        () -> {
          currentState = ShooterState.SHOOT;
          shooter.setPosition(ShooterConstants.SHOOTER_FENDER_AIM);
          hood.setHoodPosition(false);
        },
        shooter,
        hood);
  }

  public static Command setShooterHumanIntakePosition(Shooter shooter, Hood hood) {
    return Commands.runOnce(
        () -> {
          currentState = ShooterState.EMPTY;
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
    return Commands.run(
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

  public static ShooterState getShooterState() {
    return currentState;
  }

  public static void setShooterState(ShooterState state) {
    currentState = state;
  }
}
