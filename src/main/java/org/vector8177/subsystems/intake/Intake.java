package org.vector8177.subsystems.intake;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.vector8177.Constants;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SlewRateLimiter rateLimiter;

  private boolean preparingToShoot = false;
  private BooleanSupplier intakingSupp;

  public Intake(IntakeIO io, BooleanSupplier intakingSupp) {
    this.io = io;
    this.intakingSupp = intakingSupp;
    rateLimiter = new SlewRateLimiter(.4);
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
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // if (!intakingSupp.getAsBoolean())
    // {
    //   io.setFeederVoltage(0);
    //   io.setIndexerVoltage(0);
    // }
  }

  public void stop() {
    io.stop();
  }
}
