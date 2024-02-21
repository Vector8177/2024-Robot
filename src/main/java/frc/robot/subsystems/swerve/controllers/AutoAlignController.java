package frc.robot.subsystems.swerve.controllers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.LoggedTunableNumber;

public class AutoAlignController {
    private static LoggedTunableNumber thetakP = 
        new LoggedTunableNumber("AutoAlign/thetakP", Constants.SwerveConstants.AutoAlignConstants.thetaP);
    private static LoggedTunableNumber thetakD = 
        new LoggedTunableNumber("AutoAlign/thetakP", Constants.SwerveConstants.AutoAlignConstants.thetaD);
    private static LoggedTunableNumber thetaTolerance = 
        new LoggedTunableNumber("AutoAlign/thetaTolerance", Constants.SwerveConstants.AutoAlignConstants.thetaTolerance);
    private static LoggedTunableNumber maxAngulatVelocity = 
        new LoggedTunableNumber("AutoAlign/angularVelocity", Constants.SwerveConstants.AutoAlignConstants.maxAngularVelocity);
    private static LoggedTunableNumber maxAngularAcceleration = 
        new LoggedTunableNumber("AutoAlign/maxAngularAcceleration", Constants.SwerveConstants.AutoAlignConstants.maxAngularAcceleration);
    
    private Pose2d goalPose = null;

    private ProfiledPIDController thetaController;

    private final Swerve swerve;

    public AutoAlignController(Pose2d goalPose, Swerve swerve)
    {
        this.goalPose = goalPose;
        Logger.recordOutput("AutoAlign/goalPose", goalPose);

        this.swerve = swerve;

        thetaController = new ProfiledPIDController(thetakP.get(), 0, thetakD.get(), new TrapezoidProfile.Constraints(maxAngulatVelocity.get(), maxAngularAcceleration.get()));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(thetaTolerance.get());

        Pose2d currentPose = swerve.getPose();
    }
    
}
