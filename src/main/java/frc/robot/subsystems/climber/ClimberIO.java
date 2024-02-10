package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    double leftClimberVelocityRadPerSec = 0d;
    double rightClimberVelocityRadPerSec = 0d;
    double leftClimberAppliedVolts = 0d;
    double rightClimberAppliedVolts = 0d;
    double[] leftClimberCurrentAmps = new double[] {};
    double[] rightClimberCurrentAmps = new double[] {};
    double leftClimberAbsoluteEncoderPosition = 0d;
    double rightClimberAbsoluteEncoderPosition = 0d;
    double leftClimberEncoderPosition = 0d;
    double rightClimberEncoderPosition = 0d;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setLeftClimberVoltage(double volts) {}

  default void setRightClimberVoltage(double volts) {}

  default void stop() {}
}
