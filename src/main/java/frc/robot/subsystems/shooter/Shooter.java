package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.PivotUnweightedPIDConstants;
import frc.robot.Constants.ShooterConstants.ShooterUnweightedPIDConstants;

public class Shooter extends SubsystemBase {
  private double targetPosition;
  private double targetTopSpeed;
  private double targetBottomSpeed;

  private final PIDController pivotPidController;
  private ArmFeedforward pivotFeedForward;

  private final PIDController shooterTopSpeedPidController;
  private final PIDController shooterBottomSpeedPidController;

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;

    pivotPidController = new PIDController(PivotUnweightedPIDConstants.P.getVal(),
        PivotUnweightedPIDConstants.I.getVal(), PivotUnweightedPIDConstants.D.getVal());
    pivotPidController.enableContinuousInput(0, Math.PI * 2);
    pivotPidController.setTolerance(ShooterConstants.PIVOT_TOLERANCE);

    pivotFeedForward = new ArmFeedforward(
        PivotUnweightedPIDConstants.S.getVal(),
        PivotUnweightedPIDConstants.G.getVal(),
        PivotUnweightedPIDConstants.V.getVal(),
        PivotUnweightedPIDConstants.A.getVal());

    shooterTopSpeedPidController = new PIDController(ShooterUnweightedPIDConstants.P.getVal(),
        ShooterUnweightedPIDConstants.I.getVal(), ShooterUnweightedPIDConstants.D.getVal());
    shooterTopSpeedPidController.setTolerance(ShooterConstants.SHOOTER_SPEED_TOLERANCE);

    shooterBottomSpeedPidController = new PIDController(ShooterUnweightedPIDConstants.P.getVal(),
        ShooterUnweightedPIDConstants.I.getVal(), ShooterUnweightedPIDConstants.D.getVal());
    shooterBottomSpeedPidController.setTolerance(ShooterConstants.SHOOTER_SPEED_TOLERANCE);
  }

  public void setTargetShooterSpeed(double topSpeed, double bottomSpeed) {
    // TODO
    targetTopSpeed = topSpeed;
    targetBottomSpeed = bottomSpeed;
  }

  public void setTargetShooterPosition(double rad) {
    // TODO
    targetPosition = rad;
  }

  public void setTargetShooterIndexer(double speed) {
    // TODO
    io.setShooterIndexerVoltage(speed);
  }

  public double getShooterTopFixedVelocity() {
    return inputs.shooterTopFixedVelocityRadPerSec;
  }

  public double getShooterBottomFixedVelocity() {
    return inputs.shooterBottomFixedVelocityRadPerSec;
  }

  public double getShooterPositionVelocity() {
    return inputs.shooterPivotVelocityRadPerSec;
  }

  public double getShooterIndexerVelocity() {
    return inputs.shooterIndexerVelocityRadPerSec;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    double pivotMotorSpeed = 
      pivotPidController.calculate(inputs.shooterPivotAbsolutePosition.getRadians(),targetPosition)
        + pivotFeedForward.calculate(inputs.shooterPivotAbsolutePosition.getRadians(), 0);
      
    io.setShooterPositionVoltage(
      MathUtil.clamp(pivotMotorSpeed, -ShooterConstants.MAX_MOTOR_VOLTAGE, ShooterConstants.MAX_MOTOR_VOLTAGE)
    );

    double shooterTopSpeed = 
      shooterTopSpeedPidController.calculate(inputs.shooterTopFixedVelocityRadPerSec, targetTopSpeed);
    double shooterBottomSpeed = 
      shooterBottomSpeedPidController.calculate(inputs.shooterBottomFixedVelocityRadPerSec, targetBottomSpeed);
    
    io.setShooterSpeedVoltage(
      MathUtil.clamp(shooterTopSpeed, -ShooterConstants.MAX_MOTOR_VOLTAGE, ShooterConstants.MAX_MOTOR_VOLTAGE), 
      MathUtil.clamp(shooterBottomSpeed, -ShooterConstants.MAX_MOTOR_VOLTAGE, ShooterConstants.MAX_MOTOR_VOLTAGE)
    );
  }

  public void stop() {
    io.stop();
  }
}
