package org.vector8177.subsystems.intake;

import org.vector8177.Constants.IntakeConstants;
import org.vector8177.util.SparkUtils;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax intakeLeftFeederSparkMax;
  private final RelativeEncoder intakeLeftFeederEncoder;
  private final CANSparkMax intakeRightFeederSparkMax;
  private final RelativeEncoder intakeRightFeederEncoder;
  private final CANSparkMax intakeIndexerSparkMax;
  private final RelativeEncoder intakeIndexerEncoder;

  public IntakeIOSparkMax() {
    intakeLeftFeederSparkMax =
        new CANSparkMax(IntakeConstants.LEFT_FEEDER_MOTOR_ID, MotorType.kBrushless);
    intakeRightFeederSparkMax =
        new CANSparkMax(IntakeConstants.RIGHT_FEEDER_MOTOR_ID, MotorType.kBrushless);
    intakeIndexerSparkMax = new CANSparkMax(IntakeConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

    intakeLeftFeederSparkMax.restoreFactoryDefaults();
    intakeLeftFeederSparkMax.setCANTimeout(250);
    intakeRightFeederSparkMax.restoreFactoryDefaults();
    intakeRightFeederSparkMax.setCANTimeout(250);
    intakeIndexerSparkMax.restoreFactoryDefaults();
    intakeIndexerSparkMax.setCANTimeout(250);

    intakeLeftFeederSparkMax.setSmartCurrentLimit(40);
    intakeRightFeederSparkMax.setSmartCurrentLimit(40);
    intakeIndexerSparkMax.setSmartCurrentLimit(35);

    intakeLeftFeederEncoder = intakeLeftFeederSparkMax.getEncoder();
    intakeLeftFeederSparkMax.enableVoltageCompensation(12d);
    intakeRightFeederEncoder = intakeLeftFeederSparkMax.getEncoder();
    intakeRightFeederSparkMax.enableVoltageCompensation(12d);
    intakeIndexerEncoder = intakeLeftFeederSparkMax.getEncoder();
    intakeIndexerSparkMax.enableVoltageCompensation(12d);

    SparkUtils.configureNothingFrameStrategy(intakeIndexerSparkMax);
    SparkUtils.configureNothingFrameStrategy(intakeLeftFeederSparkMax);
    SparkUtils.configureNothingFrameStrategy(intakeRightFeederSparkMax);

    intakeLeftFeederEncoder.setMeasurementPeriod(60);
    intakeIndexerEncoder.setMeasurementPeriod(60);

    intakeLeftFeederSparkMax.setIdleMode(IdleMode.kCoast);
    intakeRightFeederSparkMax.setIdleMode(IdleMode.kCoast);

    intakeRightFeederSparkMax.follow(intakeLeftFeederSparkMax);
    intakeRightFeederSparkMax.setInverted(true);
    intakeIndexerSparkMax.setInverted(true);

    intakeLeftFeederSparkMax.burnFlash();
    intakeRightFeederSparkMax.burnFlash();
    intakeIndexerSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeLeftFeederAppliedVolts =
        intakeLeftFeederSparkMax.getAppliedOutput() * intakeLeftFeederSparkMax.getBusVoltage();
    // inputs.intakeLeftFeederVelocityRadPerSec =
    //     Units.rotationsPerMinuteToRadiansPerSecond(intakeLeftFeederEncoder.getVelocity());
    // inputs.intakeLeftFeederCurrentAmps = new double[]
    // {intakeLeftFeederSparkMax.getOutputCurrent()};

    inputs.intakeRightFeederAppliedVolts =
        intakeRightFeederSparkMax.getAppliedOutput() * intakeRightFeederSparkMax.getBusVoltage();
    // inputs.intakeRightFeederVelocityRadPerSec =
    //     Units.rotationsPerMinuteToRadiansPerSecond(intakeRightFeederEncoder.getVelocity());
    // inputs.intakeRightFeederCurrentAmps =
    //     new double[] {intakeRightFeederSparkMax.getOutputCurrent()};

    inputs.intakeIndexerAppliedVolts =
        intakeIndexerSparkMax.getAppliedOutput() * intakeIndexerSparkMax.getBusVoltage();
    // inputs.intakeIndexerVelocityRadPerSec =
    //     Units.rotationsPerMinuteToRadiansPerSecond(intakeIndexerEncoder.getVelocity());
    // inputs.intakeIndexerCurrentAmps = new double[] {intakeIndexerSparkMax.getOutputCurrent()};
  }

  @Override
  public void setFeederVoltage(double volts) {
    intakeLeftFeederSparkMax.setVoltage(volts);
    intakeRightFeederSparkMax.setVoltage(-volts);
  }

  @Override
  public void setIndexerVoltage(double volts) {
    intakeIndexerSparkMax.setVoltage(volts);
  }

  @Override
  public void stop() {
    intakeLeftFeederSparkMax.setVoltage(0);
    intakeIndexerSparkMax.setVoltage(0);
  }
}
