package org.vector8177.subsystems.shooter;

import org.vector8177.Constants.ShooterConstants;
import org.vector8177.util.SparkUtils;
import org.vector8177.util.SparkUtils.Data;
import org.vector8177.util.SparkUtils.Sensor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSparkMax implements ShooterIO {
  private final CANSparkMax shooterTopFixedSparkMax;
  private final CANSparkMax shooterPivotSparkMax;
  private final CANSparkMax shooterBottomFixedSparkMax;
  private final CANSparkMax shooterIndexerSparkMax;

  private final RelativeEncoder shooterTopFixedEncoder;
  private final RelativeEncoder shooterBottomFixedEncoder;
  private final RelativeEncoder shooterPivotEncoder;
  private final RelativeEncoder shooterIndexerEncoder;

  private final AbsoluteEncoder shooterPivotAbsoluteEncoder;

  private Rotation2d pivotRelativeOffset = new Rotation2d();

  private final AnalogInput shooterIRSensor;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 colorSensorV3;

  public ShooterIOSparkMax() {
    shooterTopFixedSparkMax =
        new CANSparkMax(ShooterConstants.SHOOTER_TOP_ID, MotorType.kBrushless);
    shooterBottomFixedSparkMax =
        new CANSparkMax(ShooterConstants.SHOOTER_BOTTOM_ID, MotorType.kBrushless);
    shooterPivotSparkMax = new CANSparkMax(ShooterConstants.SHOOTER_PIVOT_ID, MotorType.kBrushless);
    shooterIndexerSparkMax =
        new CANSparkMax(ShooterConstants.SHOOTER_INDEXER_ID, MotorType.kBrushless);

    shooterIRSensor = new AnalogInput(ShooterConstants.SHOOTER_IR_SENSOR_PORT);

    colorSensorV3 = new ColorSensorV3(i2cPort);
    colorSensorV3.configureProximitySensor(
        ProximitySensorResolution.kProxRes11bit, ProximitySensorMeasurementRate.kProxRate12ms);

    shooterTopFixedSparkMax.restoreFactoryDefaults();
    shooterBottomFixedSparkMax.restoreFactoryDefaults();
    shooterPivotSparkMax.restoreFactoryDefaults();
    shooterIndexerSparkMax.restoreFactoryDefaults();

    shooterTopFixedSparkMax.setCANTimeout(250);
    shooterBottomFixedSparkMax.setCANTimeout(250);
    shooterPivotSparkMax.setCANTimeout(250);
    shooterIndexerSparkMax.setCANTimeout(250);

    shooterTopFixedSparkMax.setSmartCurrentLimit(24);
    shooterBottomFixedSparkMax.setSmartCurrentLimit(24);
    shooterPivotSparkMax.setSmartCurrentLimit(20);
    shooterIndexerSparkMax.setSmartCurrentLimit(35);

    shooterTopFixedEncoder = shooterTopFixedSparkMax.getEncoder();
    shooterBottomFixedEncoder = shooterBottomFixedSparkMax.getEncoder();
    shooterPivotEncoder = shooterPivotSparkMax.getEncoder();
    shooterIndexerEncoder = shooterIndexerSparkMax.getEncoder();

    shooterBottomFixedSparkMax.setIdleMode(IdleMode.kCoast);
    shooterTopFixedSparkMax.setIdleMode(IdleMode.kCoast);

    shooterPivotAbsoluteEncoder = shooterPivotSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    shooterPivotAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI);
    shooterPivotAbsoluteEncoder.setZeroOffset(ShooterConstants.ABSOLUTE_OFFSET);

    shooterPivotEncoder.setPosition(shooterPivotAbsoluteEncoder.getPosition());
    shooterPivotEncoder.setPositionConversionFactor(2 * Math.PI);

    shooterBottomFixedSparkMax.setInverted(true);

    Logger.recordOutput("Shooter/PivotRelativeOffset", pivotRelativeOffset);
    Logger.recordOutput("Shooter/Absolute", shooterPivotAbsoluteEncoder.getPosition());
    Logger.recordOutput("Shooter/Relative", shooterPivotEncoder.getPosition());

    shooterTopFixedSparkMax.enableVoltageCompensation(12);
    shooterBottomFixedSparkMax.enableVoltageCompensation(12);
    shooterPivotSparkMax.enableVoltageCompensation(12);
    shooterIndexerSparkMax.enableVoltageCompensation(12);

    shooterTopFixedEncoder.setMeasurementPeriod(60);
    shooterBottomFixedEncoder.setMeasurementPeriod(60);
    shooterPivotEncoder.setMeasurementPeriod(60);
    shooterIndexerEncoder.setMeasurementPeriod(60);

    SparkUtils.configureFrameStrategy(
        shooterTopFixedSparkMax,
        Set.of(Data.VELOCITY, Data.APPLIED_OUTPUT),
        Set.of(Sensor.INTEGRATED),
        true);
    SparkUtils.configureNothingFrameStrategy(shooterBottomFixedSparkMax);
    SparkUtils.configureFrameStrategy(
        shooterPivotSparkMax,
        Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
        Set.of(Sensor.ABSOLUTE),
        false);
    SparkUtils.configureFrameStrategy(
        shooterIndexerSparkMax, Set.of(Data.APPLIED_OUTPUT), Set.of(), false);

    // shooterBottomFixedSparkMax.follow(shooterTopFixedSparkMax, true);

    shooterTopFixedSparkMax.burnFlash();
    shooterBottomFixedSparkMax.burnFlash();
    shooterPivotSparkMax.burnFlash();
    shooterIndexerSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterTopFixedAppliedVolts =
        shooterTopFixedSparkMax.getAppliedOutput() * shooterTopFixedSparkMax.getBusVoltage();
    inputs.shooterTopFixedRPM = shooterTopFixedEncoder.getVelocity();
    // inputs.shooterTopFixedCurrentAmps = new double[]
    // {shooterTopFixedSparkMax.getOutputCurrent()};

    inputs.shooterBottomFixedAppliedVolts =
        shooterBottomFixedSparkMax.getAppliedOutput() * shooterBottomFixedSparkMax.getBusVoltage();
    inputs.shooterBottomFixedRPM = shooterBottomFixedEncoder.getVelocity();
    // inputs.shooterBottomFixedCurrentAmps =
    //     new double[] {shooterBottomFixedSparkMax.getOutputCurrent()};

    inputs.shooterPivotAppliedVolts =
        shooterPivotSparkMax.getAppliedOutput() * shooterPivotSparkMax.getBusVoltage();
    inputs.shooterPivotVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(shooterPivotEncoder.getVelocity());
    inputs.shooterPivotCurrentAmps = new double[] {shooterPivotSparkMax.getOutputCurrent()};

    inputs.shooterIndexerAppliedVolts =
        shooterIndexerSparkMax.getAppliedOutput() * shooterIndexerSparkMax.getBusVoltage();
    inputs.shooterIndexerVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(shooterIndexerEncoder.getVelocity());
    inputs.shooterIndexerCurrentAmps = new double[] {shooterIndexerSparkMax.getOutputCurrent()};

    inputs.shooterPivotRelativePosition =
        (Rotation2d.fromRadians(shooterPivotAbsoluteEncoder.getPosition()));
    inputs.shooterPivotAbsolutePosition =
        Rotation2d.fromRadians(shooterPivotAbsoluteEncoder.getPosition());

    inputs.shooterSensorTriggerVoltage = shooterIRSensor.getVoltage();

    Color colorDetected = colorSensorV3.getColor();
    inputs.colorDetected =
        new double[] {colorDetected.red, colorDetected.blue, colorDetected.green};
    inputs.proximity = colorSensorV3.getProximity();
    inputs.irOutputRaw = colorSensorV3.getIR();
  }

  @Override
  public void setShooterSpeedVoltage(double top, double bottom) {
    shooterTopFixedSparkMax.setVoltage(top);
    shooterBottomFixedSparkMax.setVoltage(bottom);
  }

  @Override
  public void setShooterPositionVoltage(double volts) {
    shooterPivotSparkMax.setVoltage(volts);
  }

  @Override
  public void setShooterIndexerVoltage(double volts) {
    shooterIndexerSparkMax.setVoltage(volts);
  }

  @Override
  public void stop() {
    shooterTopFixedSparkMax.setVoltage(0d);
    shooterBottomFixedSparkMax.setVoltage(0d);
    shooterPivotSparkMax.setVoltage(0d);
    shooterIndexerSparkMax.setVoltage(0d);
  }
}
