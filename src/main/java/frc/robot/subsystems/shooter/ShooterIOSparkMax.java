package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;

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

  public ShooterIOSparkMax() {
    shooterTopFixedSparkMax =
        new CANSparkMax(ShooterConstants.SHOOTER_TOP_ID, MotorType.kBrushless);
    shooterBottomFixedSparkMax =
        new CANSparkMax(ShooterConstants.SHOOTER_BOTTOM_ID, MotorType.kBrushless);
    shooterPivotSparkMax = new CANSparkMax(ShooterConstants.SHOOTER_PIVOT_ID, MotorType.kBrushless);
    shooterIndexerSparkMax =
        new CANSparkMax(ShooterConstants.SHOOTER_INDEXER_ID, MotorType.kBrushless);

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
    shooterIndexerSparkMax.setSmartCurrentLimit(20);

    shooterTopFixedEncoder = shooterTopFixedSparkMax.getEncoder();
    shooterBottomFixedEncoder = shooterBottomFixedSparkMax.getEncoder();
    shooterPivotEncoder = shooterPivotSparkMax.getEncoder();
    shooterIndexerEncoder = shooterIndexerSparkMax.getEncoder();

    shooterPivotAbsoluteEncoder = shooterPivotSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    shooterPivotAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI);
    shooterPivotAbsoluteEncoder.setZeroOffset(ShooterConstants.ABSOLUTE_OFFSET);
    shooterPivotEncoder.setPosition(shooterPivotAbsoluteEncoder.getPosition());
    shooterPivotEncoder.setPositionConversionFactor(2 * Math.PI);

    shooterTopFixedSparkMax.enableVoltageCompensation(12);
    shooterBottomFixedSparkMax.enableVoltageCompensation(12);
    shooterPivotSparkMax.enableVoltageCompensation(12);
    shooterIndexerSparkMax.enableVoltageCompensation(12);

    shooterTopFixedEncoder.setMeasurementPeriod(50);
    shooterBottomFixedEncoder.setMeasurementPeriod(50);
    shooterPivotEncoder.setMeasurementPeriod(50);
    shooterIndexerEncoder.setMeasurementPeriod(50);

    shooterTopFixedSparkMax.burnFlash();
    shooterBottomFixedSparkMax.burnFlash();
    shooterPivotSparkMax.burnFlash();
    shooterIndexerSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterTopFixedAppliedVolts =
        shooterTopFixedSparkMax.getAppliedOutput() * shooterTopFixedSparkMax.getBusVoltage();
    inputs.shooterTopFixedVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(shooterTopFixedEncoder.getVelocity());
    inputs.shooterTopFixedCurrentAmps = new double[] {shooterTopFixedSparkMax.getOutputCurrent()};

    inputs.shooterBottomFixedAppliedVolts =
        shooterBottomFixedSparkMax.getAppliedOutput() * shooterBottomFixedSparkMax.getBusVoltage();
    inputs.shooterBottomFixedVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(shooterBottomFixedEncoder.getVelocity());
    inputs.shooterBottomFixedCurrentAmps =
        new double[] {shooterBottomFixedSparkMax.getOutputCurrent()};

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

    inputs.shooterOffset = ShooterConstants.ABSOLUTE_OFFSET;
    inputs.shooterPivotRelativePosition = Rotation2d.fromRadians(shooterPivotEncoder.getPosition());
    inputs.shooterPivotAbsolutePosition =
        Rotation2d.fromRadians(shooterPivotAbsoluteEncoder.getPosition());
  }

  @Override
  public void setShooterSpeedVoltage(double topVolts, double bottomVolts) {
    shooterTopFixedSparkMax.setVoltage(topVolts);
    shooterBottomFixedSparkMax.setVoltage(bottomVolts);
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
