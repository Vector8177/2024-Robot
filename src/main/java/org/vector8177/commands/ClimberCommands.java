package org.vector8177.commands;

import org.vector8177.Constants.ClimberConstants;
import org.vector8177.subsystems.climber.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.BooleanSupplier;

public class ClimberCommands {
  private ClimberCommands() {}

  public static Command runClimber(
      Climber climber,
      BooleanSupplier leftIncrease,
      BooleanSupplier leftDecrease,
      BooleanSupplier rightIncrease,
      BooleanSupplier rightDecrease) {
    return Commands.run(
        () -> {
          double leftPoisitionVal = 0d;
          double rightPositionVal = 0d;
          if (leftIncrease.getAsBoolean()) {
            leftPoisitionVal += ClimberConstants.climberSpeed;
          }
          if (leftDecrease.getAsBoolean()) {
            leftPoisitionVal -= ClimberConstants.climberSpeed;
          }
          if (rightIncrease.getAsBoolean()) {
            rightPositionVal += ClimberConstants.climberSpeed;
          }
          if (rightDecrease.getAsBoolean()) {
            rightPositionVal -= ClimberConstants.climberSpeed;
          }
          climber.setLeftClimberPosition(leftPoisitionVal + climber.getLeftClimberPosition());
          climber.setRightClimberPosition(rightPositionVal + climber.getRightClimberPosition());
        },
        climber);
  }
}
