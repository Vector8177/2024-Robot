package frc.robot.subsystems.swerve.controllers;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.LoggedTunableNumber;
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

  private Pose2d goalPose = null;

  private ProfiledPIDController thetaController;

  private final Swerve swerve;

  public AutoAlignController(Pose2d goalPose, Swerve swerve) {
    this.goalPose = goalPose;

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
    Pose2d currentPose = swerve.getPose();
    Rotation2d rotationToTarget =
        goalPose.getTranslation().minus(currentPose.getTranslation()).getAngle();

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
    double distToTarget =
        Units.inchesToMeters(goalPose.getTranslation().getDistance(currentPose.getTranslation()));
    Logger.recordOutput("AutoAlign/ShooterPose/Distance", distToTarget);
    double returnVal =
        Math.PI / 2
            + Math.atan((Constants.SPEAKER_HEIGHT - Constants.SHOOTER_HEIGHT) / distToTarget);
    Logger.recordOutput(
        "AutoAlign/ShooterPose/ShooterTargetPose", Units.radiansToDegrees(returnVal));

    return returnVal;
  }
}
