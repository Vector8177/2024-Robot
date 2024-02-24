package frc.robot.subsystems.hood;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

public class HoodIOSparkMax implements HoodIO {
  private final CANSparkMax hoodPivotSparkMax;
  private final RelativeEncoder hoodPivotEncoder;

  public HoodIOSparkMax() {
    hoodPivotSparkMax = new CANSparkMax(HoodConstants.HOOD_MOTOR_ID, MotorType.kBrushless);

    hoodPivotSparkMax.restoreFactoryDefaults();
    hoodPivotSparkMax.setCANTimeout(250);

    hoodPivotEncoder = hoodPivotSparkMax.getEncoder();
    hoodPivotEncoder.setPosition(0);
    hoodPivotEncoder.setPositionConversionFactor(
        2 * Math.PI * Constants.HoodConstants.HOOD_GEAR_RATIO);
    hoodPivotSparkMax.enableVoltageCompensation(12d);

    hoodPivotEncoder.setMeasurementPeriod(60);

    hoodPivotSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodPivotAppliedVolts =
        hoodPivotSparkMax.getAppliedOutput() * hoodPivotSparkMax.getBusVoltage();
    inputs.hoodPivotVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(hoodPivotEncoder.getVelocity());
    inputs.hoodPivotCurrentAmps = new double[] {hoodPivotSparkMax.getOutputCurrent()};
    inputs.hoodPivotEncoderPosition = Rotation2d.fromRadians(hoodPivotEncoder.getPosition());
  }

  @Override
  public void setHoodPivotVoltage(double volts) {
    hoodPivotSparkMax.setVoltage(volts);
  }

  @Override
  public void stop() {
    hoodPivotSparkMax.setVoltage(0);
  }
}
