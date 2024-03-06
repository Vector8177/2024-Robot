package org.vector8177.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.vector8177.Constants.ShooterConstants;
import org.vector8177.util.SparkUtils;
import org.vector8177.util.SparkUtils.Data;
import org.vector8177.util.SparkUtils.Sensor;

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

  public ShooterIOSparkMax() {
    shooterTopFixedSparkMax =
        new CANSparkMax(ShooterConstants.SHOOTER_TOP_ID, MotorType.kBrushless);
    shooterBottomFixedSparkMax =
        new CANSparkMax(ShooterConstants.SHOOTER_BOTTOM_ID, MotorType.kBrushless);
    shooterPivotSparkMax = new CANSparkMax(ShooterConstants.SHOOTER_PIVOT_ID, MotorType.kBrushless);
    shooterIndexerSparkMax =
        new CANSparkMax(ShooterConstants.SHOOTER_INDEXER_ID, MotorType.kBrushless);

    shooterIRSensor = new AnalogInput(ShooterConstants.SHOOTER_IR_SENSOR_PORT);

    shooterTopFixedSparkMax.restoreFactoryDefaults();
    shooterBottomFixedSparkMax.restoreFactoryDefaults();
    shooterPivotSparkMax.restoreFactoryDefaults();
    shooterIndexerSparkMax.restoreFactoryDefaults();

    shooterTopFixedSparkMax.setCANTimeout(250);
    shooterBottomFixedSparkMax.setCANTimeout(250);
    shooterPivotSparkMax.setCANTimeout(250);
    shooterIndexerSparkMax.setCANTimeout(250);

    shooterTopFixedSparkMax.setSmartCurrentLimit(20);
    shooterBottomFixedSparkMax.setSmartCurrentLimit(20);
    shooterPivotSparkMax.setSmartCurrentLimit(20);
    shooterIndexerSparkMax.setSmartCurrentLimit(35);

    shooterTopFixedEncoder = shooterTopFixedSparkMax.getEncoder();
    shooterBottomFixedEncoder = shooterBottomFixedSparkMax.getEncoder();
    shooterPivotEncoder = shooterPivotSparkMax.getEncoder();
    shooterIndexerEncoder = shooterIndexerSparkMax.getEncoder();

    shooterPivotAbsoluteEncoder = shooterPivotSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    shooterPivotAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI);
    shooterPivotAbsoluteEncoder.setZeroOffset(ShooterConstants.ABSOLUTE_OFFSET);

    shooterPivotEncoder.setPosition(shooterPivotAbsoluteEncoder.getPosition());
    shooterPivotEncoder.setPositionConversionFactor(2 * Math.PI);

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
    SparkUtils.configureNothingFrameStrategy(shooterIndexerSparkMax);

    shooterBottomFixedSparkMax.follow(shooterTopFixedSparkMax, true);

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
    // inputs.shooterIndexerVelocityRadPerSec =
    //     Units.rotationsPerMinuteToRadiansPerSecond(shooterIndexerEncoder.getVelocity());
    // inputs.shooterIndexerCurrentAmps = new double[] {shooterIndexerSparkMax.getOutputCurrent()};

    inputs.shooterPivotRelativePosition =
        (Rotation2d.fromRadians(shooterPivotAbsoluteEncoder.getPosition()));
    inputs.shooterPivotAbsolutePosition =
        Rotation2d.fromRadians(shooterPivotAbsoluteEncoder.getPosition());

    inputs.shooterSensorTriggerVoltage = shooterIRSensor.getVoltage();
  }

  @Override
  public void setShooterSpeedVoltage(double volts) {
    shooterTopFixedSparkMax.setVoltage(volts);
    shooterBottomFixedSparkMax.setVoltage(-volts);
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
