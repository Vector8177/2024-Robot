package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
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

  // private final SysIdRoutine sysId;
  // private final SimpleMotorFeedforward shooterSpeedFeedForward;

  private MechanismRoot2d rootMech;
  private MechanismLigament2d m_shooter;

  private Rotation2d targetPosition = new Rotation2d();

  private Double wheelTargetSpeed = 0d;

  public Shooter(ShooterIO io, Mechanism2d mainMech) {
    this.io = io;

    pivotPidController =
        new PIDController(
            ShooterConstants.SHOOTER_PIVOT_KP,
            ShooterConstants.SHOOTER_PIVOT_KI,
            ShooterConstants.SHOOTER_PIVOT_KD);
    pivotPidController.enableContinuousInput(0, 2 * Math.PI);
    pivotPidController.setTolerance(ShooterConstants.PIVOT_TOLERANCE);

    shooterSpeedPidController =
        new PIDController(
            ShooterConstants.SHOOTER_TOP_KP,
            ShooterConstants.SHOOTER_TOP_KI,
            ShooterConstants.SHOOTER_TOP_KD);
    // wheelTargetSpeed);

    // shooterSpeedFeedForward = (1 / ShooterConstants.SHOOTER_FF_V) * wheelTargetSpeed
    shooterSpeedPidController.setTolerance(ShooterConstants.SHOOTER_SPEED_TOLERANCE);

    // pivotRelativeOffset =
    // inputs.shooterPivotAbsolutePosition.minus(inputs.shooterPivotRelativePosition);

    rootMech = mainMech.getRoot("shooter", 5, 1.5);
    m_shooter =
        rootMech.append(
            new MechanismLigament2d(
                "shooterPivot", 2, inputs.shooterPivotRelativePosition.getDegrees()));
    // sysId =
    // new SysIdRoutine(
    // new Config(
    // null,
    // null,
    // null,
    // (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
    // new SysIdRoutine.Mechanism(
    // (voltage) -> runCharacterization(voltage.in(Volts)),
    // (log) ->
    // log.motor("shooter-wheel")
    // .voltage(Volts.of(inputs.shooterTopFixedAppliedVolts))
    // .angularVelocity(
    // Units.RotationsPerSecond.of(inputs.shooterTopFixedRPM * 60)),
    // this));
  }

  public void setShooterSpeed(double speed) {
    wheelTargetSpeed = speed;
  }

  public void setShooterRawVolts(double volts) {
    io.setShooterSpeedVoltage(volts);
  }

  public void setPosition(double rad) {
    targetPosition = Rotation2d.fromRadians(rad);
  }

  public void runCharacterization(double volts) {
    wheelTargetSpeed = null;

    io.setShooterSpeedVoltage(volts);
  }

  public void setIndexerSpeed(double speed) {
    io.setShooterIndexerVoltage(speed * ShooterConstants.MAX_MOTOR_VOLTAGE);
  }

  public void disableClosedLoop() {

    wheelTargetSpeed = null;
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

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  // return sysId.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  // return sysId.dynamic(direction);
  // }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/SetPoints", targetPosition);

    // Logger.recordOutput("Shooter/SetPoints/WheelTargetSpeed", wheelTargetSpeed);

    m_shooter.setAngle(inputs.shooterPivotRelativePosition);

    double pivotMotorSpeed =
        pivotPidController.calculate(
            inputs.shooterPivotRelativePosition.getRadians(), targetPosition.getRadians());

    io.setShooterPositionVoltage(
        MathUtil.clamp(
            pivotMotorSpeed,
            -ShooterConstants.MAX_MOTOR_VOLTAGE,
            ShooterConstants.MAX_MOTOR_VOLTAGE));

    if (wheelTargetSpeed != null) {
      double shooterSpeed =
          shooterSpeedPidController.calculate(inputs.shooterTopFixedRPM, wheelTargetSpeed);
      double shooterFF = (1 / ShooterConstants.SHOOTER_FF_V) * wheelTargetSpeed;
      // DriverStation.reportWarning(
      // "TOP: " + shooterTopSpeed + "; BOTTOM: " + shooterBottomSpeed, false);
      // double shooterFF =
      //     (wheelTargetSpeed - inputs.shooterTopFixedRPM) / ShooterConstants.SHOOTER_FF_V;

      // Logger.recordOutput("Shooter/FF_Val", shooterFF);
      // Logger.recordOutput("Shooter/PID_Val", shooterSpeed);

      io.setShooterSpeedVoltage(
          MathUtil.clamp(
              shooterSpeed + shooterFF,
              -ShooterConstants.MAX_MOTOR_VOLTAGE,
              ShooterConstants.MAX_MOTOR_VOLTAGE));
    }
  }

  public void stop() {
    io.stop();
  }
}
