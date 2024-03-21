// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
// Barghav is a yapper
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.vector8177;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static org.vector8177.commands.MainCommands.*;

import org.vector8177.Constants.ShooterConstants;
import org.vector8177.Constants.ShooterState;
import org.vector8177.Constants.SwerveConstants.DriveMode;
import org.vector8177.Constants.VisionConstants;
import org.vector8177.commands.SwerveCommands;
import org.vector8177.subsystems.climber.Climber;
import org.vector8177.subsystems.climber.ClimberIO;
import org.vector8177.subsystems.climber.ClimberIOSim;
import org.vector8177.subsystems.climber.ClimberIOSparkMax;
import org.vector8177.subsystems.hood.Hood;
import org.vector8177.subsystems.hood.HoodIO;
import org.vector8177.subsystems.hood.HoodIOSim;
import org.vector8177.subsystems.hood.HoodIOSparkMax;
import org.vector8177.subsystems.intake.Intake;
import org.vector8177.subsystems.intake.IntakeIO;
import org.vector8177.subsystems.intake.IntakeIOSim;
import org.vector8177.subsystems.intake.IntakeIOSparkMax;
import org.vector8177.subsystems.shooter.Shooter;
import org.vector8177.subsystems.shooter.ShooterIO;
import org.vector8177.subsystems.shooter.ShooterIOSim;
import org.vector8177.subsystems.shooter.ShooterIOSparkMax;
import org.vector8177.subsystems.swerve.GyroIO;
import org.vector8177.subsystems.swerve.GyroIOPigeon2;
import org.vector8177.subsystems.swerve.ModuleIO;
import org.vector8177.subsystems.swerve.ModuleIOSim;
import org.vector8177.subsystems.swerve.ModuleIOSparkMax;
import org.vector8177.subsystems.swerve.Swerve;
import org.vector8177.subsystems.vision.AprilTagVisionIO;
import org.vector8177.subsystems.vision.AprilTagVisionIOReal;
import org.vector8177.subsystems.vision.AprilTagVisionIOSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Swerve swerve;
  private final Shooter shooter;
  private final Intake intake;
  private final Hood hood;
  // private final Vision vision;
  private int n = 0; // TODO: why is there a variable called n?Ishaaaaaaaaaaaan
  private final Climber climber;

  private DriveMode currentDriveMode = DriveMode.TELEOP;

  private boolean isIntaking = false;

  private final Mechanism2d mainMech = new Mechanism2d(10, 10);

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // vision = new Vision(new CameraIOPhoton(VisionConstants.frontLeftCameraName));
        swerve =
            new Swerve(
                new GyroIOPigeon2(false),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3),
                new AprilTagVisionIOReal(),
                () -> currentDriveMode);
        shooter =
            new Shooter(
                new ShooterIOSparkMax(),
                () -> currentDriveMode,
                () -> Rotation2d.fromRadians(swerve.calculateAngleAutoAlign()),
                () -> isIntaking,
                () -> swerve.calculateSpeedAutoAlign(),
                mainMech);
        intake = new Intake(new IntakeIOSparkMax(), () -> shooter.getShooterOccupied());
        hood = new Hood(new HoodIOSparkMax(), shooter.getMechanismLigament2d());
        climber = new Climber(new ClimberIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // vision = null;
        swerve =
            new Swerve(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new AprilTagVisionIOSim(),
                () -> currentDriveMode);
        shooter =
            new Shooter(
                new ShooterIOSim(),
                () -> currentDriveMode,
                () -> Rotation2d.fromRadians(swerve.calculateAngleAutoAlign()),
                () -> isIntaking,
                () -> swerve.calculateSpeedAutoAlign(),
                mainMech);
        intake = new Intake(new IntakeIOSim(), () -> shooter.getShooterOccupied());
        hood = new Hood(new HoodIOSim(), shooter.getMechanismLigament2d());
        climber = new Climber(new ClimberIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        // vision = new Vision(new CameraIO() {});
        swerve =
            new Swerve(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new AprilTagVisionIO() {},
                () -> currentDriveMode);
        shooter =
            new Shooter(
                new ShooterIO() {},
                () -> currentDriveMode,
                () -> Rotation2d.fromRadians(swerve.calculateAngleAutoAlign()),
                () -> isIntaking,
                () -> swerve.calculateSpeedAutoAlign(),
                mainMech);
        intake = new Intake(new IntakeIO() {}, () -> shooter.getShooterOccupied());
        hood = new Hood(new HoodIO() {}, shooter.getMechanismLigament2d());
        climber = new Climber(new ClimberIO() {});
        break;
    }

    NamedCommands.registerCommand("Enable AutoAlign", setAutoAlign(true));
    NamedCommands.registerCommand("Disable AutoAlign", setAutoAlign(false));
    NamedCommands.registerCommand("Toggle AutoAlign", toggleAutoAlign());
    NamedCommands.registerCommand("Run Shoot Sequence", runShootSequenceAuto(shooter));
    NamedCommands.registerCommand("Run Intake Sequence", runIntake(intake, shooter, hood));
    NamedCommands.registerCommand("Fender Shot Position", setShooterShootPosition(shooter, hood));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // autoChooser.addOption(
    // "Shooter SysId (Quasistatic Forward)",
    // shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Shooter SysId (Quasistatic Reverse)",
    // shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Shooter SysId (Dynamic Forward)",
    // shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Shooter SysId (Dynamic Reverse)",
    // shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    swerve.setDefaultCommand(
        SwerveCommands.joystickDrive(
            swerve,
            shooter,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            driverController.leftBumper()));

    climber.setDefaultCommand(
        runClimber(
            climber,
            operatorController.povUp(),
            operatorController.leftBumper(),
            operatorController.povUp(),
            operatorController.rightBumper()));

    driverController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        swerve.setPose(
                            new Pose2d(swerve.getPose().getTranslation(), new Rotation2d())),
                    swerve)
                .ignoringDisable(true));

    driverController.b().onTrue(toggleAutoAlign());

    driverController.a().onTrue(runOnce(() -> swerve.useVision = !swerve.useVision));

    operatorController.povDown().onTrue(setShooterShootPosition(shooter, hood));

    operatorController.a().onTrue(runShooter(shooter));

    operatorController.rightTrigger().onTrue(runIntake(intake, shooter, hood));
    operatorController.rightTrigger().onFalse(stopIntake(intake, shooter));

    operatorController.leftTrigger().whileTrue(runOuttake(intake, shooter));

    operatorController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.setShooterSpeed(0);
                },
                shooter));

    operatorController.povDown().onTrue(setShooterIntakePosition(shooter, hood));

    driverController.povDown().onTrue(runOnce(() -> swerve.useVision = !swerve.useVision));

    operatorController.povRight().onTrue(setShooterAmpPosition(shooter, hood));

    operatorController.povLeft().onTrue(setShooterShootPosition(shooter, hood));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void updateMech() {
    Logger.recordOutput("Mechanism", mainMech);
    Logger.recordOutput("TeleopCommands/ShooterState", shooter.currentState);
    Logger.recordOutput("TeleopCommands/IntakeState", getCurrentIntakeState());
    Logger.recordOutput("AutoAlign/AutoAlignMode", currentDriveMode);
    Logger.recordOutput("Constants/CamOffsetLeft", VisionConstants.frontLeftCameraPosition);
    Logger.recordOutput("Constants/CamOffsetRight", VisionConstants.frontRightCameraPosition);
    // Logger.recordOutput("Shooter/ShooterOccupied", shooter.getShooterOccupied());
  }

  public Command toggleAutoAlign() {
    return Commands.runOnce(
        () -> {
          currentDriveMode =
              currentDriveMode == DriveMode.TELEOP ? DriveMode.AUTO_ALIGN : DriveMode.TELEOP;
          if (currentDriveMode == DriveMode.AUTO_ALIGN) {
            shooter.currentState = ShooterState.SHOOT;
          }
        });
  }

  public Command setAutoAlign(boolean autoAlign) {
    return Commands.runOnce(
        () -> {
          // System.out.println("pew! pew!");

          DriverStation.reportError("Changing Auto Align to " + autoAlign, false);
          currentDriveMode = autoAlign ? DriveMode.AUTO_ALIGN : DriveMode.TELEOP;

          if (currentDriveMode == DriveMode.AUTO_ALIGN) {
            shooter.currentState = (ShooterState.SHOOT);
          }
          DriverStation.reportError("Changed Auto Align to " + autoAlign, false);
        });
  }

  public void setIntakingState(boolean iState) {
    this.isIntaking = iState;
  }

  public void enableVision() {
    swerve.useVision = true;
  }

  public void disableVision() {
    swerve.useVision = true;
  }

  public Command basicAuto() {

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              shooter.setPosition(ShooterConstants.SHOOTER_FENDER_AIM);
              hood.setHoodPosition(false);
            },
            shooter,
            hood),
        runShootSequence(shooter));
  }
}
