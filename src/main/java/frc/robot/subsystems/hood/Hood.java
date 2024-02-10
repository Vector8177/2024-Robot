package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final PIDController pivotController;

  private double desiredPivotPosition;

  public Hood(HoodIO io) {
    this.io = io;

    pivotController =
        new PIDController(
            Constants.HoodConstants.hoodPivotKP,
            Constants.HoodConstants.hoodPivotKI,
            Constants.HoodConstants.hoodPivotKD);
  }

  public void setHoodPosition(double rad) {
    desiredPivotPosition = rad;
  }

  public double getPivotVelocity() {
    return inputs.hoodPivotVelocityRadPerSec;
  }

  @Override
  public void periodic() {
    io.setHoodPivotVoltage(
        MathUtil.clamp(
            pivotController.calculate(inputs.hoodPivotEncoderPosition, desiredPivotPosition),
            -Constants.HoodConstants.maxHoodMotorVoltage,
            Constants.HoodConstants.maxHoodMotorVoltage));
  }

  public void stop() {
    io.stop();
  }
}
