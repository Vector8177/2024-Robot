package org.vector8177.subsystems.climber;

import org.vector8177.Constants.ClimberConstants;

import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class ClimberIOSparkMax implements ClimberIO {
  private final CANSparkMax leftClimberSparkMax;
  private final RelativeEncoder leftClimberEncoder;
  // private final SparkAbsoluteEncoder leftClimberAbsoluteEncoder;
  private final CANSparkMax rightClimberSparkMax;
  private final RelativeEncoder rightClimberEncoder;
  // private final SparkAbsoluteEncoder rightClimberAbsoluteEncoder;

  public ClimberIOSparkMax() {
    leftClimberSparkMax = new CANSparkMax(ClimberConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightClimberSparkMax = new CANSparkMax(ClimberConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    leftClimberSparkMax.restoreFactoryDefaults();
    leftClimberSparkMax.setCANTimeout(250);
    rightClimberSparkMax.restoreFactoryDefaults();
    rightClimberSparkMax.setCANTimeout(250);

    // leftClimberAbsoluteEncoder = leftClimberSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    // rightClimberAbsoluteEncoder = rightClimberSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    leftClimberEncoder = leftClimberSparkMax.getEncoder();
    leftClimberSparkMax.enableVoltageCompensation(12d);
    rightClimberEncoder = rightClimberSparkMax.getEncoder();
    rightClimberSparkMax.enableVoltageCompensation(12d);

    leftClimberSparkMax.setSmartCurrentLimit(40);
    rightClimberSparkMax.setSmartCurrentLimit(40);

    leftClimberEncoder.setMeasurementPeriod(50);
    rightClimberEncoder.setMeasurementPeriod(50);

    leftClimberEncoder.setPosition(0);
    rightClimberEncoder.setPosition(0);

    leftClimberSparkMax.setInverted(true);
    leftClimberSparkMax.setIdleMode(IdleMode.kBrake);
    rightClimberSparkMax.setIdleMode(IdleMode.kBrake);
    // rightClimberSparkMax.setInverted(true);

    leftClimberSparkMax.burnFlash();
    rightClimberSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftClimberAppliedVolts =
        leftClimberSparkMax.getAppliedOutput() * leftClimberSparkMax.getBusVoltage();
    inputs.leftClimberVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(leftClimberEncoder.getVelocity());
    inputs.leftClimberCurrentAmps = new double[] {leftClimberSparkMax.getOutputCurrent()};
    inputs.leftClimberEncoderPosition = leftClimberEncoder.getPosition();
    // inputs.leftClimberAbsoluteEncoderPosition = inputs.leftClimberEncoderPosition;

    inputs.rightClimberAppliedVolts =
        rightClimberSparkMax.getAppliedOutput() * rightClimberSparkMax.getBusVoltage();
    inputs.rightClimberVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(rightClimberEncoder.getVelocity());
    inputs.rightClimberCurrentAmps = new double[] {rightClimberSparkMax.getOutputCurrent()};
    inputs.rightClimberEncoderPosition = rightClimberEncoder.getPosition();
    // inputs.rightClimberAbsoluteEncoderPosition = inputs.rightClimberEncoderPosition;
  }

  @Override
  public void setLeftClimberVoltage(double volts) {
    leftClimberSparkMax.setVoltage(volts);
  }

  @Override
  public void setRightClimberVoltage(double volts) {
    rightClimberSparkMax.setVoltage(volts);
  }

  @Override
  public void stop() {
    leftClimberSparkMax.setVoltage(0);
    rightClimberSparkMax.setVoltage(0);
  }
}
