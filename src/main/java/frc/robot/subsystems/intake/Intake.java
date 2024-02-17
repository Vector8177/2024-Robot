package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setIndexerSpeed(double speed) {
    io.setIndexerVoltage(speed * Constants.IntakeConstants.maxIntakeMotorVoltage);
  }

  public void setFeederSpeed(double speed) {
    io.setFeederVoltage(speed * Constants.IntakeConstants.maxIntakeMotorVoltage);
  }

  public double getFeederVelocity() {
    return inputs.intakeIndexerVelocityRadPerSec;
  }

  @Override
  public void periodic() {}

  public void stop() {
    io.stop();
  }
}
