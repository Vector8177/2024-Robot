package org.vector8177.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    double hoodPivotVelocityRadPerSec = 0d;
    double hoodPivotAppliedVolts = 0d;
    double[] hoodPivotCurrentAmps = new double[] {};
    Rotation2d hoodPivotEncoderPosition = new Rotation2d();
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void setHoodPivotVoltage(double volts) {}

  default void stop() {}
}
