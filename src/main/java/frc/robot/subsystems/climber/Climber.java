package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final PIDController climberController;

  private double leftClimberRelativeOffset;
  private double desiredLeftClimberPosition;
  private double rightClimberRelativeOffset;
  private double desiredRightClimberPosition;

  public Climber(ClimberIO io) {
    this.io = io;

    climberController =
        new PIDController(
            Constants.ClimberConstants.climberKP,
            Constants.ClimberConstants.climberKI,
            Constants.ClimberConstants.climberKD);

    leftClimberRelativeOffset =
        inputs.leftClimberAbsoluteEncoderPosition - inputs.leftClimberEncoderPosition;
    rightClimberRelativeOffset =
        inputs.rightClimberAbsoluteEncoderPosition - inputs.rightClimberEncoderPosition;
  }

  public void setLeftClimberPosition(double meters) {
    if (meters > ClimberConstants.climberTopLimit) {
      desiredLeftClimberPosition = ClimberConstants.climberTopLimit;
    } else if (meters < ClimberConstants.climberBottomLimit) {
      desiredLeftClimberPosition = ClimberConstants.climberBottomLimit;
    } else {
      desiredLeftClimberPosition = meters;
    }
  }

  public void setRightClimberPosition(double meters) {
    if (meters > ClimberConstants.climberTopLimit) {
      desiredRightClimberPosition = ClimberConstants.climberTopLimit;
    } else if (meters < ClimberConstants.climberBottomLimit) {
      desiredRightClimberPosition = ClimberConstants.climberBottomLimit;
    } else {
      desiredRightClimberPosition = meters;
    }
  }

  public double getRightClimberPosition() {
    return inputs.rightClimberEncoderPosition + rightClimberRelativeOffset;
  }

  public double getLeftClimberPosition() {
    return inputs.leftClimberEncoderPosition + leftClimberRelativeOffset;
  }

  public double getLeftClimberVelocity() {
    return inputs.leftClimberVelocityRadPerSec;
  }

  public double getRightClimberVelocity() {
    return inputs.rightClimberVelocityRadPerSec;
  }

  @Override
  public void periodic() {
    io.setLeftClimberVoltage(
        MathUtil.clamp(
            climberController.calculate(
                inputs.leftClimberEncoderPosition + leftClimberRelativeOffset,
                desiredLeftClimberPosition),
            -Constants.ClimberConstants.maxClimberMotorVoltage,
            Constants.ClimberConstants.maxClimberMotorVoltage));

    io.setRightClimberVoltage(
        MathUtil.clamp(
            climberController.calculate(
                inputs.rightClimberEncoderPosition + rightClimberRelativeOffset,
                desiredRightClimberPosition),
            -Constants.ClimberConstants.maxClimberMotorVoltage,
            Constants.ClimberConstants.maxClimberMotorVoltage));
  }

  public void stop() {
    io.stop();
  }
}
