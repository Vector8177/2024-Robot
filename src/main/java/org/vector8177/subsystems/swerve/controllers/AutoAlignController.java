package org.vector8177.subsystems.swerve.controllers;

import org.vector8177.Constants;
import org.vector8177.subsystems.swerve.Swerve;
import org.vector8177.util.LoggedTunableNumber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class AutoAlignController {
  private static LoggedTunableNumber thetakP =
      new LoggedTunableNumber(
          "AutoAlign/thetakP", Constants.SwerveConstants.AutoAlignConstants.thetaP);
  private static LoggedTunableNumber thetakD =
      new LoggedTunableNumber(
          "AutoAlign/thetakP", Constants.SwerveConstants.AutoAlignConstants.thetaD);
  private static LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber(
          "AutoAlign/thetaTolerance", Constants.SwerveConstants.AutoAlignConstants.thetaTolerance);
  private static LoggedTunableNumber maxAngularVelocity =
      new LoggedTunableNumber(
          "AutoAlign/angularVelocity",
          Constants.SwerveConstants.AutoAlignConstants.maxAngularVelocity);
  private static LoggedTunableNumber maxAngularAcceleration =
      new LoggedTunableNumber(
          "AutoAlign/maxAngularAcceleration",
          Constants.SwerveConstants.AutoAlignConstants.maxAngularAcceleration);

  private static final double RED_X = 16.58;
  private static final double BLUE_X = -.0381;
  private static final double POSE_Y = 5.55;

  private Pose2d goalPose = new Pose2d(.24, 5.55, Rotation2d.fromRotations(0));

  private ProfiledPIDController thetaController;

  private final Swerve swerve;

  private BooleanSupplier isRedSupp;

  public AutoAlignController(BooleanSupplier isRedAlliance, Swerve swerve) {
    this.isRedSupp = isRedAlliance;
    this.swerve = swerve;

    thetaController =
        new ProfiledPIDController(
            thetakP.get(),
            0,
            thetakD.get(),
            new TrapezoidProfile.Constraints(
                maxAngularVelocity.get(), maxAngularAcceleration.get()));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(thetaTolerance.get());

    Pose2d currentPose = swerve.getPose();
    ChassisSpeeds fieldVel = swerve.getSpeeds();

    Rotation2d rotationToTarget =
        goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    thetaController.reset(currentPose.getRotation().getRadians(), fieldVel.omegaRadiansPerSecond);

    thetaController.setGoal(goalPose.getRotation().getRadians());
    Logger.recordOutput("AutoAlign/goalPose", goalPose);
  }

  public double updateDrive() {
    this.goalPose =
        new Pose2d(
            (isRedSupp.getAsBoolean() ? RED_X : BLUE_X), POSE_Y, Rotation2d.fromRotations(0));
    Pose2d currentPose = swerve.getPose();
    Rotation2d rotationToTarget =
        goalPose
            .getTranslation()
            .minus(currentPose.getTranslation())
            .getAngle()
            .plus(Rotation2d.fromDegrees(180));

    Logger.recordOutput(
        "AutoAlign/RotationTarget", new Pose2d(currentPose.getTranslation(), rotationToTarget));

    double angularVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(), rotationToTarget.getRadians())
            + thetaController.getSetpoint().velocity;

    Logger.recordOutput("AutoAlign/Setpoint", angularVelocity);

    return angularVelocity;
  }

  public double updateAngle() {
    Pose2d currentPose = swerve.getPose();
    this.goalPose =
        new Pose2d(
            (isRedSupp.getAsBoolean() ? RED_X : BLUE_X), POSE_Y, Rotation2d.fromRotations(0));

    Translation2d targ = goalPose.getTranslation();
    Logger.recordOutput("AutoAlign/Trans", targ);

    Logger.recordOutput("AutoAlign/Shooter/Current", currentPose);
    Logger.recordOutput("AutoAlign/Shooter/Target", goalPose);
    double distToTarget = goalPose.getTranslation().getDistance(currentPose.getTranslation());
    Logger.recordOutput("AutoAlign/ShooterPose/Distance", distToTarget);
    double returnVal =
        Rotation2d.fromDegrees(90).getRadians()
            + Math.atan((Constants.SPEAKER_HEIGHT - Constants.SHOOTER_HEIGHT) / distToTarget);
    Logger.recordOutput(
        "AutoAlign/ShooterPose/ShooterTargetPoseRaw",
        Units.radiansToDegrees(
            Math.atan((Constants.SPEAKER_HEIGHT - Constants.SHOOTER_HEIGHT) / distToTarget)));

    Logger.recordOutput(
        "AutoAlign/ShooterPose/ShooterTargetPose", Units.radiansToDegrees(returnVal));

    return returnVal;
  }
}
