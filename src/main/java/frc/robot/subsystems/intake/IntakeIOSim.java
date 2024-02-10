package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotorSim intakeLeftSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.025);
  private DCMotorSim intakeRightSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.025);
  private DCMotorSim intakeIndexerSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.004);

  private double intakeLeftAppliedVolts = 0d;
  private double intakeRightAppliedVolts = 0d;
  private double intakeIndexerAppliedVolts = 0d;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeLeftSim.update(LOOP_PERIOD_SECS);
    intakeRightSim.update(LOOP_PERIOD_SECS);
    intakeIndexerSim.update(LOOP_PERIOD_SECS);

    inputs.intakeLeftAppliedVolts = intakeLeftAppliedVolts;
    inputs.intakeLeftVelocityRadPerSec = intakeLeftSim.getAngularVelocityRadPerSec();
    inputs.intakeLeftCurrentAmps = new double[] {Math.abs(intakeLeftSim.getCurrentDrawAmps())};

    inputs.intakeRightAppliedVolts = intakeRightAppliedVolts;
    inputs.intakeRightVelocityRadPerSec = intakeRightSim.getAngularVelocityRadPerSec();
    inputs.intakeRightCurrentAmps = new double[] {Math.abs(intakeRightSim.getCurrentDrawAmps())};

    inputs.intakeIndexerAppliedVolts = intakeIndexerAppliedVolts;
    inputs.intakeIndexerVelocityRadPerSec = intakeIndexerSim.getAngularVelocityRadPerSec();
    inputs.intakeIndexerCurrentAmps =
        new double[] {Math.abs(intakeIndexerSim.getCurrentDrawAmps())};
  }

  @Override
  public void setPositionVoltage(double volts) {
    intakeLeftAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    intakeRightAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    intakeLeftSim.setInputVoltage(intakeLeftAppliedVolts);
    intakeRightSim.setInputVoltage(intakeRightAppliedVolts);
  }

  @Override
  public void setIndexerVoltage(double volts) {
    intakeIndexerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    intakeIndexerSim.setInputVoltage(intakeIndexerAppliedVolts);
  }

  @Override
  public void stop() {
    intakeLeftAppliedVolts = 0;
    intakeRightAppliedVolts = 0;
    intakeIndexerAppliedVolts = 0;

    intakeLeftSim.setInputVoltage(intakeLeftAppliedVolts);
    intakeRightSim.setInputVoltage(intakeRightAppliedVolts);
    intakeIndexerSim.setInputVoltage(intakeIndexerAppliedVolts);
  }
}
