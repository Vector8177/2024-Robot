package org.vector8177.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotorSim leftClimberSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);
  private DCMotorSim rightClimberSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);

  private double leftClimberAppliedVolts = 0d;
  private double rightClimberAppliedVolts = 0d;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    leftClimberSim.update(LOOP_PERIOD_SECS);
    rightClimberSim.update(LOOP_PERIOD_SECS);

    inputs.leftClimberAppliedVolts = leftClimberAppliedVolts;
    inputs.leftClimberVelocityRadPerSec = leftClimberSim.getAngularVelocityRadPerSec();
    inputs.leftClimberCurrentAmps = new double[] {Math.abs(leftClimberSim.getCurrentDrawAmps())};

    inputs.rightClimberAppliedVolts = rightClimberAppliedVolts;
    inputs.rightClimberVelocityRadPerSec = rightClimberSim.getAngularVelocityRadPerSec();
    inputs.rightClimberCurrentAmps = new double[] {Math.abs(rightClimberSim.getCurrentDrawAmps())};
  }

  @Override
  public void setLeftClimberVoltage(double volts) {
    leftClimberAppliedVolts = MathUtil.clamp(volts, -12d, 12d);
    leftClimberSim.setInputVoltage(leftClimberAppliedVolts);
  }

  @Override
  public void setRightClimberVoltage(double volts) {
    rightClimberAppliedVolts = MathUtil.clamp(volts, -12d, 12d);
    rightClimberSim.setInputVoltage(rightClimberAppliedVolts);
  }

  @Override
  public void stop() {
    leftClimberAppliedVolts = 0;
    rightClimberAppliedVolts = 0;

    leftClimberSim.setInputVoltage(leftClimberAppliedVolts);
    rightClimberSim.setInputVoltage(rightClimberAppliedVolts);
  }
}
