package org.vector8177.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterTopFixedRPM = 0d;
    public double shooterBottomFixedRPM = 0d;
    public double shooterPivotVelocityRadPerSec = 0d;
    public double shooterIndexerVelocityRadPerSec = 0d;

    public double shooterTopFixedAppliedVolts = 0d;
    public double shooterBottomFixedAppliedVolts = 0d;
    public double shooterPivotAppliedVolts = 0d;
    public double shooterIndexerAppliedVolts = 0d;

    public Rotation2d shooterPivotRelativePosition = new Rotation2d();
    public Rotation2d shooterPivotAbsolutePosition = new Rotation2d();

    public double[] shooterTopFixedCurrentAmps = new double[] {};
    public double[] shooterBottomFixedCurrentAmps = new double[] {};
    public double[] shooterPivotCurrentAmps = new double[] {};
    public double[] shooterIndexerCurrentAmps = new double[] {};

    public double shooterSensorTriggerVoltage = 0d;

    public double[] colorDetected = new double[3];
    public int proximity = 0;
    public double irOutputRaw = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterSpeedVoltage(double top, double bottom) {}

  public default void setShooterPositionVoltage(double volts) {}

  public default void setShooterIndexerVoltage(double volts) {}

  public default void stop() {}
}
