package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax intakeLeftSparkMax;
  private final RelativeEncoder intakeLeftEncoder;
  private final CANSparkMax intakeRightSparkMax;
  private final RelativeEncoder intakeRightEncoder;
  private final CANSparkMax intakeIndexerSparkMax;
  private final RelativeEncoder intakeIndexerEncoder;

  public IntakeIOSparkMax() {
    intakeLeftSparkMax = new CANSparkMax(Constants.placeHolderMotorID, MotorType.kBrushless);
    intakeRightSparkMax = new CANSparkMax(Constants.placeHolderMotorID, MotorType.kBrushless);
    intakeIndexerSparkMax = new CANSparkMax(Constants.placeHolderMotorID, MotorType.kBrushless);

    intakeLeftSparkMax.restoreFactoryDefaults();
    intakeLeftSparkMax.setCANTimeout(250);
    intakeRightSparkMax.restoreFactoryDefaults();
    intakeRightSparkMax.setCANTimeout(250);
    intakeIndexerSparkMax.restoreFactoryDefaults();
    intakeIndexerSparkMax.setCANTimeout(250);

    intakeLeftSparkMax.setSmartCurrentLimit(20);
    intakeRightSparkMax.setSmartCurrentLimit(20);
    intakeIndexerSparkMax.setSmartCurrentLimit(20);

    intakeLeftEncoder = intakeLeftSparkMax.getEncoder();
    intakeLeftSparkMax.enableVoltageCompensation(12d);
    intakeRightEncoder = intakeLeftSparkMax.getEncoder();
    intakeRightSparkMax.enableVoltageCompensation(12d);
    intakeIndexerEncoder = intakeLeftSparkMax.getEncoder();
    intakeIndexerSparkMax.enableVoltageCompensation(12d);

    intakeLeftEncoder.setMeasurementPeriod(50);
    intakeIndexerEncoder.setMeasurementPeriod(50);

    intakeRightSparkMax.follow(intakeLeftSparkMax);
    intakeRightSparkMax.setInverted(true);

    intakeLeftSparkMax.burnFlash();
    intakeRightSparkMax.burnFlash();
    intakeIndexerSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeLeftAppliedVolts =
        intakeLeftSparkMax.getAppliedOutput() * intakeLeftSparkMax.getBusVoltage();
    inputs.intakeLeftVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(intakeLeftEncoder.getVelocity());
    inputs.intakeLeftCurrentAmps = new double[] {intakeLeftSparkMax.getOutputCurrent()};

    inputs.intakeRightAppliedVolts =
        intakeRightSparkMax.getAppliedOutput() * intakeRightSparkMax.getBusVoltage();
    inputs.intakeRightVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(intakeRightEncoder.getVelocity());
    inputs.intakeRightCurrentAmps = new double[] {intakeRightSparkMax.getOutputCurrent()};

    inputs.intakeIndexerAppliedVolts =
        intakeIndexerSparkMax.getAppliedOutput() * intakeIndexerSparkMax.getBusVoltage();
    inputs.intakeIndexerVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(intakeIndexerEncoder.getVelocity());
    inputs.intakeIndexerCurrentAmps = new double[] {intakeIndexerSparkMax.getOutputCurrent()};

    inputs.intakeEncoderPosition =
        (intakeLeftEncoder.getPosition() + intakeRightEncoder.getPosition()) / 2d;
  }

  @Override
  public void setPositionVoltage(double volts) {
    intakeLeftSparkMax.setVoltage(volts);
  }

  @Override
  public void setIndexerVoltage(double volts) {
    intakeIndexerSparkMax.setVoltage(volts);
  }

  @Override
  public void stop() {
    intakeLeftSparkMax.setVoltage(0);
    intakeIndexerSparkMax.setVoltage(0);
  }
}
