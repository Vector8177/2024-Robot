package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim implements HoodIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotorSim hoodPivotSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.025);

  private double hoodPivotAppliedVolts = 0d;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    hoodPivotSim.update(LOOP_PERIOD_SECS);

    inputs.hoodPivotAppliedVolts = hoodPivotAppliedVolts;
    inputs.hoodPivotVelocityRadPerSec = hoodPivotSim.getAngularVelocityRadPerSec();
    inputs.hoodPivotCurrentAmps = new double[] {Math.abs(hoodPivotSim.getCurrentDrawAmps())};
  }

  @Override
  public void setHoodPivotVoltage(double volts) {
    hoodPivotAppliedVolts = MathUtil.clamp(volts, -12d, 12d);

    hoodPivotSim.setInputVoltage(hoodPivotAppliedVolts);
  }

  @Override
  public void stop() {
    hoodPivotAppliedVolts = 0d;
    hoodPivotSim.setInputVoltage(hoodPivotAppliedVolts);
  }
}
