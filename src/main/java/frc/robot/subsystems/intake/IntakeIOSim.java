package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotorSim intakeLeftFeederSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.025);
  private DCMotorSim intakeRightFeederSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.025);
  private DCMotorSim intakeIndexerSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.004);

  private double intakeLeftFeederAppliedVolts = 0d;
  private double intakeRightFeederAppliedVolts = 0d;
  private double intakeIndexerAppliedVolts = 0d;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeLeftFeederSim.update(LOOP_PERIOD_SECS);
    intakeRightFeederSim.update(LOOP_PERIOD_SECS);
    intakeIndexerSim.update(LOOP_PERIOD_SECS);

    inputs.intakeLeftFeederAppliedVolts = intakeLeftFeederAppliedVolts;
    inputs.intakeLeftFeederVelocityRadPerSec = intakeLeftFeederSim.getAngularVelocityRadPerSec();
    inputs.intakeLeftFeederCurrentAmps =
        new double[] {Math.abs(intakeLeftFeederSim.getCurrentDrawAmps())};

    inputs.intakeRightFeederAppliedVolts = intakeRightFeederAppliedVolts;
    inputs.intakeRightFeederVelocityRadPerSec = intakeRightFeederSim.getAngularVelocityRadPerSec();
    inputs.intakeRightFeederCurrentAmps =
        new double[] {Math.abs(intakeRightFeederSim.getCurrentDrawAmps())};

    inputs.intakeIndexerAppliedVolts = intakeIndexerAppliedVolts;
    inputs.intakeIndexerVelocityRadPerSec = intakeIndexerSim.getAngularVelocityRadPerSec();
    inputs.intakeIndexerCurrentAmps =
        new double[] {Math.abs(intakeIndexerSim.getCurrentDrawAmps())};
  }

  @Override
  public void setFeederVoltage(double volts) {
    intakeLeftFeederAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    intakeRightFeederAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    intakeLeftFeederSim.setInputVoltage(intakeLeftFeederAppliedVolts);
    intakeRightFeederSim.setInputVoltage(intakeRightFeederAppliedVolts);
  }

  @Override
  public void setIndexerVoltage(double volts) {
    intakeIndexerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    intakeIndexerSim.setInputVoltage(intakeIndexerAppliedVolts);
  }

  @Override
  public void stop() {
    intakeLeftFeederAppliedVolts = 0;
    intakeRightFeederAppliedVolts = 0;
    intakeIndexerAppliedVolts = 0;

    intakeLeftFeederSim.setInputVoltage(intakeLeftFeederAppliedVolts);
    intakeRightFeederSim.setInputVoltage(intakeRightFeederAppliedVolts);
    intakeIndexerSim.setInputVoltage(intakeIndexerAppliedVolts);
  }
}
