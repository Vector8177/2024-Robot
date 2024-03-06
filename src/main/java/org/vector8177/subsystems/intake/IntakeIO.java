package org.vector8177.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    double intakeLeftFeederVelocityRadPerSec = 0d;
    double intakeRightFeederVelocityRadPerSec = 0d;
    double intakeIndexerVelocityRadPerSec = 0d;
    double intakeLeftFeederAppliedVolts = 0d;
    double intakeRightFeederAppliedVolts = 0d;
    double intakeIndexerAppliedVolts = 0d;
    double[] intakeLeftFeederCurrentAmps = new double[] {};
    double[] intakeRightFeederCurrentAmps = new double[] {};
    double[] intakeIndexerCurrentAmps = new double[] {};
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setFeederVoltage(double volts) {}

  default void setIndexerVoltage(double volts) {}

  default void stop() {}
}
