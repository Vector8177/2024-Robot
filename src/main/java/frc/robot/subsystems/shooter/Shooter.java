package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final PIDController pivotPidController;
  private final PIDController shooterTopSpeedPidController;
  private final PIDController shooterBottomSpeedPidController;

  private final ArmFeedforward pivotFeedForward;

  private Rotation2d pivotRelativeOffset;
  private Rotation2d targetPosition;

  private double targetTopSpeed;
  private double targetBottomSpeed;

  public Shooter(ShooterIO io) {
    this.io = io;

    pivotPidController =
        new PIDController(
            ShooterConstants.SHOOTER_PIVOT_KP,
            ShooterConstants.SHOOTER_PIVOT_KI,
            ShooterConstants.SHOOTER_PIVOT_KD);
    pivotPidController.enableContinuousInput(0, Math.PI * 2);
    pivotPidController.setTolerance(ShooterConstants.PIVOT_TOLERANCE);

    pivotFeedForward =
        new ArmFeedforward(
            ShooterConstants.SHOOTER_ARM_KS,
            ShooterConstants.SHOOTER_ARM_KG,
            ShooterConstants.SHOOTER_ARM_KV,
            ShooterConstants.SHOOTER_ARM_KA);

    shooterTopSpeedPidController =
        new PIDController(
            ShooterConstants.SHOOTER_TOP_KP,
            ShooterConstants.SHOOTER_TOP_KI,
            ShooterConstants.SHOOTER_TOP_KD);
    shooterTopSpeedPidController.setTolerance(ShooterConstants.SHOOTER_SPEED_TOLERANCE);

    shooterBottomSpeedPidController =
        new PIDController(
            ShooterConstants.SHOOTER_BOTTOM_KP,
            ShooterConstants.SHOOTER_BOTTOM_KP,
            ShooterConstants.SHOOTER_BOTTOM_KP);
    shooterBottomSpeedPidController.setTolerance(ShooterConstants.SHOOTER_SPEED_TOLERANCE);

    pivotRelativeOffset =
        inputs.shooterPivotAbsolutePosition.minus(inputs.shooterPivotRelativePosition);
  }

  public void setShooterSpeed(double topSpeed, double bottomSpeed) {
    targetTopSpeed = topSpeed;
    targetBottomSpeed = bottomSpeed;
  }

  public void setPosition(double rad) {
    targetPosition = Rotation2d.fromRadians(rad);
  }

  public void setIndexerSpeed(double speed) {
    io.setShooterIndexerVoltage(speed);
  }

  public double getShooterTopFixedVelocity() {
    return inputs.shooterTopFixedVelocityRadPerSec;
  }

  public double getShooterBottomFixedVelocity() {
    return inputs.shooterBottomFixedVelocityRadPerSec;
  }

  public double getIRSensorVoltage() {
    return inputs.shooterSensorTriggerVoltage;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/SetPoints/TopSpeed", targetTopSpeed);
    Logger.recordOutput("Shooter/SetPoints/BottomSpeed", targetBottomSpeed);

    double pivotMotorSpeed =
        pivotPidController.calculate(
                inputs.shooterPivotRelativePosition.getRadians() + pivotRelativeOffset.getRadians(),
                targetPosition.getRadians())
            + pivotFeedForward.calculate(
                inputs.shooterPivotRelativePosition.getRadians() + pivotRelativeOffset.getRadians(),
                0);

    io.setShooterPositionVoltage(
        MathUtil.clamp(
            pivotMotorSpeed,
            -ShooterConstants.MAX_MOTOR_VOLTAGE,
            ShooterConstants.MAX_MOTOR_VOLTAGE));

    double shooterTopSpeed =
        shooterTopSpeedPidController.calculate(
            inputs.shooterTopFixedVelocityRadPerSec, targetTopSpeed);
    double shooterBottomSpeed =
        shooterBottomSpeedPidController.calculate(
            inputs.shooterBottomFixedVelocityRadPerSec, targetBottomSpeed);

    // DriverStation.reportWarning(
    //     "TOP: " + shooterTopSpeed + "; BOTTOM: " + shooterBottomSpeed, false);

    io.setShooterSpeedVoltage(
        MathUtil.clamp(
            shooterTopSpeed,
            -ShooterConstants.MAX_MOTOR_VOLTAGE,
            ShooterConstants.MAX_MOTOR_VOLTAGE),
        MathUtil.clamp(
            shooterBottomSpeed,
            -ShooterConstants.MAX_MOTOR_VOLTAGE,
            ShooterConstants.MAX_MOTOR_VOLTAGE));
  }

  public void stop() {
    io.stop();
  }
}
