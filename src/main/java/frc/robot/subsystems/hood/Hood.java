package frc.robot.subsystems.hood;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final PIDController pivotController;

  private Rotation2d desiredPivotPosition;

  public Hood(HoodIO io) {
    this.io = io;

    pivotController =
        new PIDController(
            Constants.HoodConstants.hoodPivotKP,
            Constants.HoodConstants.hoodPivotKI,
            Constants.HoodConstants.hoodPivotKD);
    pivotController.enableContinuousInput(0, Math.PI * 2);
  }

  public void setHoodPosition(boolean isHoodUp)
  {
    setHoodPositionRad(isHoodUp ? HoodConstants.TOP_POSE : HoodConstants.BOTTOM_POSE);
  }

  private void setHoodPositionRad(double rad) {
    desiredPivotPosition = Rotation2d.fromRadians(rad);
  }

  public double getPivotVelocity() {
    return inputs.hoodPivotVelocityRadPerSec;
  }

  @Override
  public void periodic() {
    io.setHoodPivotVoltage(
        MathUtil.clamp(
            pivotController.calculate(
                inputs.hoodPivotEncoderPosition.getRadians(), desiredPivotPosition.getRadians()),
            -Constants.HoodConstants.maxHoodMotorVoltage,
            Constants.HoodConstants.maxHoodMotorVoltage));
  }

  public void stop() {
    io.stop();
  }
}
