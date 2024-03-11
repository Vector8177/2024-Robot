package org.vector8177.subsystems.intake;

import org.vector8177.Constants;
import org.vector8177.Constants.IntakeState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SlewRateLimiter rateLimiter;

  private IntakeState intakeState = IntakeState.NOT_INTAKING;

  private BooleanSupplier sensorSupplier;

  private double targetIndexerSpeed = 0;
  private double targetFeederSpeed = 0;

  public Intake(IntakeIO io, BooleanSupplier sensorSupplier) {
    this.io = io;
    this.sensorSupplier = sensorSupplier;
    rateLimiter = new SlewRateLimiter(.4);
  }

  public void setIntakingState(boolean isIntaking) {
    intakeState = isIntaking ? IntakeState.INTAKING : IntakeState.NOT_INTAKING;
  }

  private void setIndexerSpeedRaw(double speed) {
    speed = MathUtil.clamp(speed, -1, 1);
    io.setIndexerVoltage(speed * Constants.IntakeConstants.maxIntakeMotorVoltage);
  }

  private void setFeederSpeedRaw(double speed) {
    speed = MathUtil.clamp(speed, -1, 1);
    io.setFeederVoltage(speed * Constants.IntakeConstants.maxIntakeMotorVoltage);
  }

  public void setIndexerSpeed(double speed) {
    targetIndexerSpeed = speed;
  }

  public void setFeederSpeed(double speed) {
    targetFeederSpeed = speed;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (sensorSupplier.getAsBoolean()) {
      targetFeederSpeed = 0;
      targetIndexerSpeed = 0;
    }

    setFeederSpeedRaw(targetFeederSpeed);
    setIndexerSpeedRaw(targetIndexerSpeed);
  }

  public void stop() {
    io.stop();
  }
}
