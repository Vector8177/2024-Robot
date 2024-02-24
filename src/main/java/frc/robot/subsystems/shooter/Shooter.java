package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final PIDController pivotPidController;
  private final PIDController shooterSpeedPidController;

  private MechanismRoot2d rootMech;
  private MechanismLigament2d m_shooter;

  private final ArmFeedforward pivotFeedForward;

  private Rotation2d targetPosition = new Rotation2d();

  private double wheelTargetSpeed = 0d;

  public Shooter(ShooterIO io, Mechanism2d mainMech) {
    this.io = io;

    pivotPidController =
        new PIDController(
            ShooterConstants.SHOOTER_PIVOT_KP,
            ShooterConstants.SHOOTER_PIVOT_KI,
            ShooterConstants.SHOOTER_PIVOT_KD);
    pivotPidController.enableContinuousInput(-Math.PI, Math.PI);
    pivotPidController.setTolerance(ShooterConstants.PIVOT_TOLERANCE);

    pivotFeedForward =
        new ArmFeedforward(
            ShooterConstants.SHOOTER_ARM_KS,
            ShooterConstants.SHOOTER_ARM_KG,
            ShooterConstants.SHOOTER_ARM_KV,
            ShooterConstants.SHOOTER_ARM_KA);

    shooterSpeedPidController =
        new PIDController(
            ShooterConstants.SHOOTER_TOP_KP,
            ShooterConstants.SHOOTER_TOP_KI,
            ShooterConstants.SHOOTER_TOP_KD);
    shooterSpeedPidController.setTolerance(ShooterConstants.SHOOTER_SPEED_TOLERANCE);

    // pivotRelativeOffset =
    //     inputs.shooterPivotAbsolutePosition.minus(inputs.shooterPivotRelativePosition);

    rootMech = mainMech.getRoot("shooter", 5, 1.5);
    m_shooter =
        rootMech.append(
            new MechanismLigament2d(
                "shooterPivot", 2, inputs.shooterPivotRelativePosition.getDegrees()));
  }

  public void setShooterSpeed(double speed) {
    wheelTargetSpeed = speed;
  }

  public void setPosition(double deg) {
    targetPosition = Rotation2d.fromDegrees(deg);
  }

  public void setIndexerSpeed(double speed) {
    io.setShooterIndexerVoltage(speed * ShooterConstants.MAX_MOTOR_VOLTAGE);
  }

  public double getShooterTopFixedVelocity() {
    return inputs.shooterTopFixedRPM;
  }

  public double getShooterBottomFixedVelocity() {
    return inputs.shooterBottomFixedRPM;
  }

  public double getIRSensorVoltage() {
    return inputs.shooterSensorTriggerVoltage;
  }

  public MechanismLigament2d getMechanismLigament2d() {
    return m_shooter;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/SetPoints", targetPosition);

    Logger.recordOutput("Shooter/SetPoints/WheelTargetSpeed", wheelTargetSpeed);

    m_shooter.setAngle(inputs.shooterPivotRelativePosition);

    double pivotMotorSpeed =
        pivotPidController.calculate(
                inputs.shooterPivotRelativePosition.getRadians(), targetPosition.getRadians())
            + pivotFeedForward.calculate(inputs.shooterPivotRelativePosition.getRadians(), 0);

    io.setShooterPositionVoltage(
        MathUtil.clamp(
            pivotMotorSpeed,
            -ShooterConstants.MAX_MOTOR_VOLTAGE,
            ShooterConstants.MAX_MOTOR_VOLTAGE));

    double shooterSpeed =
        shooterSpeedPidController.calculate(
            inputs.shooterTopFixedRPM, wheelTargetSpeed);

    // DriverStation.reportWarning(
    //     "TOP: " + shooterTopSpeed + "; BOTTOM: " + shooterBottomSpeed, false);

    io.setShooterSpeedVoltage(
        MathUtil.clamp(
            shooterSpeed,
            -ShooterConstants.MAX_MOTOR_VOLTAGE,
            ShooterConstants.MAX_MOTOR_VOLTAGE));
  }

  public void stop() {
    io.stop();
  }
}
