package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class TeleopCommands {
  public static boolean stopIntake = false;
  public static boolean readyToShoot = false;
  public static boolean ampMode = false;

  public static enum ShooterState {
    AMP,
    SHOOT,
    EMPTY
  }

  private TeleopCommands() {}

  public static Command runShooter(Shooter shooter) {
    // if (shooter.readyToShoot) {
    // return runShootSequence(shooter);
    // } else {
    //   return Commands.none();
    // }
    // return runShootSequence(shooter);
    Logger.recordOutput("TeleopCommands/Ready", readyToShoot);
    Logger.recordOutput("TeleopCommands/Amp", ampMode);
    Logger.recordOutput("TeleopCommands/readyT", readyToShoot);
    Logger.recordOutput("TeleopCommands/ampT", ampMode);
    if (readyToShoot && ampMode) {
      Logger.recordOutput("TeleopCommands/readyT", readyToShoot);
      Logger.recordOutput("TeleopCommands/ampT", ampMode);
      return runAmpSequence(shooter);
    } else if (readyToShoot) {
      Logger.recordOutput("TeleopCommands/readyT", readyToShoot);
      Logger.recordOutput("TeleopCommands/ampT", ampMode);
      return runShootSequence(shooter);
    } else {
      Logger.recordOutput("TeleopCommands/readyT", readyToShoot);
      Logger.recordOutput("TeleopCommands/ampT", ampMode);
      return Commands.none();
    }
  }

  public static Command runShootSequence(Shooter shooter) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              stopIntake = true;
              shooter.setShooterSpeed(ShooterConstants.SHOOT_WHEEL_RPM);
            },
            shooter),
        Commands.waitUntil(
            () -> shooter.getShooterTopFixedVelocity() > ShooterConstants.SHOOT_RPM_CUTOFF),
        Commands.runOnce(
            () -> {
              shooter.setIndexerSpeed(-ShooterConstants.SHOOTER_INDEXER_SPEED);
            },
            shooter),
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
              shooter.setShooterSpeed(400);
            },
            shooter),
        Commands.waitUntil(() -> shooter.getShooterTopFixedVelocity() < 500),
        Commands.runOnce(
            () -> {
              shooter.setShooterSpeed(0);
            },
            shooter));
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
              stopIntake = true;
              shooter.setIndexerSpeed(ShooterConstants.SHOOTER_INDEXER_SPEED);
            },
            shooter),
        Commands.waitSeconds(1.2),
        Commands.runOnce(
            () -> {
              stopIntake = false;
              shooter.setIndexerSpeed(0);
            },
            shooter));
  }

  public static Command runIntake(Intake intake, Shooter shooter) {
    return Commands.runEnd(
        () -> {
          if (shooter.getIRSensorVoltage() < ShooterConstants.SHOOTER_IR_TARGET_VOLTAGE
              && !stopIntake) {
            intake.setFeederSpeed(-IntakeConstants.FEEDER_SPEED);
            intake.setIndexerSpeed(-IntakeConstants.INDEXER_SPEED);
            shooter.setIndexerSpeed(-ShooterConstants.SHOOTER_INDEXER_SPEED);
          }
        },
        () -> {
          if (!stopIntake) {
            intake.setFeederSpeed(0);
            intake.setIndexerSpeed(0);
            shooter.setIndexerSpeed(0);
          }
        },
        intake);
  }

  public static Command runOuttake(Intake intake, Shooter shooter) {
    return Commands.runEnd(
        () -> {
          if (!stopIntake) {
            intake.setFeederSpeed(IntakeConstants.FEEDER_SPEED);
            intake.setIndexerSpeed(IntakeConstants.INDEXER_SPEED);
            shooter.setIndexerSpeed(ShooterConstants.SHOOTER_INDEXER_SPEED);
          }
        },
        () -> {
          if (!stopIntake) {
            intake.setFeederSpeed(0);
            intake.setIndexerSpeed(0);
            shooter.setIndexerSpeed(0);
          }
        },
        intake);
  }

  public static Command setShooterIntakePosition(Shooter shooter, Hood hood) {
    readyToShoot = false;
    Logger.recordOutput("TeleopCommands/readyT", readyToShoot);
    Logger.recordOutput("TeleopCommands/ampT", ampMode);
    return Commands.runOnce(
        () -> {
          Logger.recordOutput("TeleopCommands/readyT", readyToShoot);
          Logger.recordOutput("TeleopCommands/ampT", ampMode);
          shooter.setPosition(ShooterConstants.SHOOTER_PIVOT_INTAKE_POSITION);
          hood.setHoodPosition(false);
        },
        shooter,
        hood);
  }

  public static Command setShooterAmpPosition(Shooter shooter, Hood hood) {
    ampMode = true;
    readyToShoot = true;
    Logger.recordOutput("TeleopCommands/readyT", readyToShoot);
    Logger.recordOutput("TeleopCommands/ampT", ampMode);
    return Commands.runOnce(
        () -> {
          Logger.recordOutput("TeleopCommands/readyT", readyToShoot);
          Logger.recordOutput("TeleopCommands/ampT", ampMode);
          shooter.setPosition(ShooterConstants.SHOOTER_PIVOT_AMP_POSITION);
          hood.setHoodPosition(true);
        },
        shooter,
        hood);
  }

  public static Command setShooterShootPosition(Shooter shooter, Hood hood) {
    ampMode = false;
    readyToShoot = true;
    Logger.recordOutput("TeleopCommands/readyT", readyToShoot);
    Logger.recordOutput("TeleopCommands/ampT", ampMode);
    return Commands.runOnce(
        () -> {
          Logger.recordOutput("TeleopCommands/readyT", readyToShoot);
          Logger.recordOutput("TeleopCommands/ampT", ampMode);
          shooter.setPosition(ShooterConstants.SHOOTER_LONG_SHOT);
          hood.setHoodPosition(false);
        },
        shooter,
        hood);
  }
}
