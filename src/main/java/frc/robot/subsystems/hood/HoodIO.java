package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    double hoodPivotVelocityRadPerSec = 0d;
    double hoodPivotAppliedVolts = 0d;
    double[] hoodPivotCurrentAmps = new double[] {};
    double hoodPivotEncoderPosition = 0d;
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void setHoodPivotVoltage(double volts) {}

  default void stop() {}
}
