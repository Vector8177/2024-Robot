package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOSparkMax implements ClimberIO {
  private final CANSparkMax leftClimberSparkMax;
  private final RelativeEncoder leftClimberEncoder;
  private final SparkAbsoluteEncoder leftClimberAbsoluteEncoder;
  private final CANSparkMax rightClimberSparkMax;
  private final RelativeEncoder rightClimberEncoder;
  private final SparkAbsoluteEncoder rightClimberAbsoluteEncoder;

  public ClimberIOSparkMax() {
    leftClimberSparkMax = new CANSparkMax(ClimberConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightClimberSparkMax = new CANSparkMax(ClimberConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    leftClimberSparkMax.restoreFactoryDefaults();
    leftClimberSparkMax.setCANTimeout(250);
    rightClimberSparkMax.restoreFactoryDefaults();
    rightClimberSparkMax.setCANTimeout(250);

    leftClimberAbsoluteEncoder = leftClimberSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    rightClimberAbsoluteEncoder = rightClimberSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    leftClimberEncoder = leftClimberSparkMax.getEncoder();
    leftClimberSparkMax.enableVoltageCompensation(12d);
    rightClimberEncoder = rightClimberSparkMax.getEncoder();
    rightClimberSparkMax.enableVoltageCompensation(12d);

    leftClimberEncoder.setMeasurementPeriod(50);

    rightClimberSparkMax.setInverted(true);

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
    inputs.leftClimberAbsoluteEncoderPosition =
        ((leftClimberAbsoluteEncoder.getPosition() * 2 * Math.PI)
                - ClimberConstants.leftClimberAbsoluteEncoderOffset)
            / 25d;
    inputs.leftClimberEncoderPosition = leftClimberAbsoluteEncoder.getPosition();

    inputs.rightClimberAppliedVolts =
        rightClimberSparkMax.getAppliedOutput() * rightClimberSparkMax.getBusVoltage();
    inputs.rightClimberVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(rightClimberEncoder.getVelocity());
    inputs.rightClimberCurrentAmps = new double[] {rightClimberSparkMax.getOutputCurrent()};
    inputs.rightClimberAbsoluteEncoderPosition =
        ((rightClimberAbsoluteEncoder.getPosition() * 2 * Math.PI)
                - ClimberConstants.rightClimberAbsoluteEncoderOffset)
            / 25d;
    inputs.rightClimberEncoderPosition = rightClimberEncoder.getPosition();
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
