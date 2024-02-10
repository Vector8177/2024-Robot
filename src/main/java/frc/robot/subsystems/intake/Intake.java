package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final PIDController positionController;
  private double desiredIntakePosition;

  public Intake(IntakeIO io) {
    this.io = io;

    positionController =
        new PIDController(
            Constants.IntakeConstants.intakePositionKP,
            Constants.IntakeConstants.intakePositionKI,
            Constants.IntakeConstants.intakePositionKD);
  }

  public void setIndexerSpeed(double speed) {
    io.setIndexerVoltage(speed * Constants.IntakeConstants.maxIntakeMotorVoltage);
  }

  public void setIntakePosition(double rad) {
    desiredIntakePosition = rad;
  }

  public double getFeederVelocity() {
    return inputs.intakeIndexerVelocityRadPerSec;
  }

  public double getPosition() {
    return inputs.intakeEncoderPosition;
  }

  @Override
  public void periodic() {
    io.setPositionVoltage(
        MathUtil.clamp(
            positionController.calculate(inputs.intakeEncoderPosition, desiredIntakePosition),
            -Constants.IntakeConstants.maxIntakeMotorVoltage,
            Constants.IntakeConstants.maxIntakeMotorVoltage));
  }

  public void stop() {
    io.stop();
  }
}
