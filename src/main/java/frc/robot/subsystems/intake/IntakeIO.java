package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    double intakeLeftVelocityRadPerSec = 0d;
    double intakeRightVelocityRadPerSec = 0d;
    double intakeIndexerVelocityRadPerSec = 0d;
    double intakeLeftAppliedVolts = 0d;
    double intakeRightAppliedVolts = 0d;
    double intakeIndexerAppliedVolts = 0d;
    double[] intakeLeftCurrentAmps = new double[] {};
    double[] intakeRightCurrentAmps = new double[] {};
    double[] intakeIndexerCurrentAmps = new double[] {};
    double intakeEncoderPosition = 0d;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setPositionVoltage(double volts) {}

  default void setIndexerVoltage(double volts) {}

  default void stop() {}
}
