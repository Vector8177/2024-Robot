package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotorSim shooterTopSim = new DCMotorSim(DCMotor.getNEO(1), 1 / 1.5, 0.025);
  private DCMotorSim shooterBottomSim = new DCMotorSim(DCMotor.getNEO(1), 1 / 1.5, 0.025);
  private DCMotorSim shooterPivotSim =
      new DCMotorSim(DCMotor.getNEO(1), Constants.ShooterConstants.SHOOTER_PIVOT_GEAR_RATIO, 0.004);
  private DCMotorSim shooterIndexerSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.004);

  private double shooterTopAppliedVolts = 0d;
  private double shooterBottomAppliedVolts = 0d;
  private double shooterPivotAppliedVolts = 0d;
  private double shooterIndexerAppliedVolts = 0d;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    shooterTopSim.update(LOOP_PERIOD_SECS);
    shooterBottomSim.update(LOOP_PERIOD_SECS);
    shooterPivotSim.update(LOOP_PERIOD_SECS);
    shooterIndexerSim.update(LOOP_PERIOD_SECS);

    inputs.shooterTopFixedAppliedVolts = shooterTopAppliedVolts;
    inputs.shooterTopFixedVelocityRadPerSec = shooterTopSim.getAngularVelocityRadPerSec();
    inputs.shooterTopFixedCurrentAmps = new double[] {Math.abs(shooterTopSim.getCurrentDrawAmps())};

    inputs.shooterBottomFixedAppliedVolts = shooterBottomAppliedVolts;
    inputs.shooterBottomFixedVelocityRadPerSec = shooterBottomSim.getAngularVelocityRadPerSec();
    inputs.shooterBottomFixedCurrentAmps =
        new double[] {Math.abs(shooterBottomSim.getCurrentDrawAmps())};

    inputs.shooterPivotAppliedVolts = shooterPivotAppliedVolts;
    inputs.shooterPivotVelocityRadPerSec = shooterPivotSim.getAngularVelocityRadPerSec();
    inputs.shooterPivotCurrentAmps = new double[] {Math.abs(shooterPivotSim.getCurrentDrawAmps())};

    inputs.shooterIndexerAppliedVolts = shooterIndexerAppliedVolts;
    inputs.shooterIndexerVelocityRadPerSec = shooterIndexerSim.getAngularVelocityRadPerSec();
    inputs.shooterIndexerCurrentAmps =
        new double[] {Math.abs(shooterIndexerSim.getCurrentDrawAmps())};

    inputs.shooterPivotRelativePosition =
        Rotation2d.fromRadians(shooterPivotSim.getAngularPositionRad());
    inputs.shooterPivotAbsolutePosition = inputs.shooterPivotRelativePosition;
  }

  @Override
  public void setShooterSpeedVoltage(double topVolts, double bottomVolts) {
    shooterTopAppliedVolts = MathUtil.clamp(topVolts, -12.0, 12.0);
    shooterBottomAppliedVolts = MathUtil.clamp(bottomVolts, -12.0, 12.0);

    shooterTopSim.setInputVoltage(shooterTopAppliedVolts);
    shooterBottomSim.setInputVoltage(shooterBottomAppliedVolts);
  }

  @Override
  public void setShooterPositionVoltage(double volts) {
    shooterPivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    shooterPivotSim.setInputVoltage(shooterPivotAppliedVolts);
  }

  @Override
  public void setShooterIndexerVoltage(double volts) {
    shooterIndexerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    shooterIndexerSim.setInputVoltage(shooterIndexerAppliedVolts);
  }

  @Override
  public void stop() {
    shooterTopAppliedVolts = 0;
    shooterBottomAppliedVolts = 0;
    shooterPivotAppliedVolts = 0;
    shooterIndexerAppliedVolts = 0;

    shooterTopSim.setInputVoltage(shooterTopAppliedVolts);
    shooterBottomSim.setInputVoltage(shooterBottomAppliedVolts);
    shooterPivotSim.setInputVoltage(shooterPivotAppliedVolts);
    shooterIndexerSim.setInputVoltage(shooterIndexerAppliedVolts);
  }
}
