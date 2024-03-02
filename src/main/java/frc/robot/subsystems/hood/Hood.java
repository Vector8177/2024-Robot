package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final PIDController pivotController;

  private MechanismLigament2d m_hood;

  private Rotation2d desiredPivotPosition = new Rotation2d(Constants.HoodConstants.SHOOT_POSE);

  public Hood(HoodIO io, MechanismLigament2d shooterLig) {
    this.io = io;
    m_hood =
        shooterLig.append(
            new MechanismLigament2d("Hood", 0.5, 0, 3, new Color8Bit(Color.kPapayaWhip)));

    pivotController =
        new PIDController(
            Constants.HoodConstants.hoodPivotKP,
            Constants.HoodConstants.hoodPivotKI,
            Constants.HoodConstants.hoodPivotKD);
    pivotController.enableContinuousInput(0, Math.PI * 2);
  }

  public void setHoodPosition(boolean amp) {
    setHoodPositionRad(amp ? HoodConstants.AMP_POSE : HoodConstants.SHOOT_POSE);
  }

  public void setHoodPosition(double pose) {
    setHoodPositionRad((pose));
  }

  private void setHoodPositionRad(double rad) {
    desiredPivotPosition = Rotation2d.fromRadians(rad);
  }

  public double getPivotVelocity() {
    return inputs.hoodPivotVelocityRadPerSec;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    Logger.recordOutput("Hood/Setpoints/Rotation", desiredPivotPosition);

    m_hood.setAngle(inputs.hoodPivotEncoderPosition);

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
