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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SwerveConstants.DriveMode;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SwerveCommands;
import frc.robot.commands.TeleopCommands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.ModuleIOSparkMax;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPhoton;
import frc.robot.subsystems.vision.Vision;
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
  private DriveMode currentMode = DriveMode.TELEOP;
  private final Swerve swerve;
  private final Shooter shooter;
  private final Intake intake;
  private final Hood hood;
  private final Vision vision;
  private int n = 0;
  // private final Climber climber;

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
        vision = new Vision(new CameraIOPhoton(VisionConstants.frontLeftCameraName));
        swerve =
            new Swerve(
                new GyroIOPigeon2(false),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3),
                vision);
        shooter = new Shooter(new ShooterIOSparkMax(), mainMech);
        intake = new Intake(new IntakeIOSparkMax());
        hood = new Hood(new HoodIOSparkMax(), shooter.getMechanismLigament2d());
        // climber = new Climber(new ClimberIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        vision = new Vision(new CameraIO() {});
        swerve =
            new Swerve(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                vision);
        shooter = new Shooter(new ShooterIOSim(), mainMech);
        intake = new Intake(new IntakeIOSim());
        hood = new Hood(new HoodIOSim(), shooter.getMechanismLigament2d());
        // climber = new Climber(new ClimberIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        vision = new Vision(new CameraIO() {});
        swerve =
            new Swerve(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                vision);
        shooter = new Shooter(new ShooterIO() {}, mainMech);
        intake = new Intake(new IntakeIO() {});
        hood = new Hood(new HoodIO() {}, shooter.getMechanismLigament2d());
        // climber = new Climber(new ClimberIO() {});
        break;
    }

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
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> currentMode));
    driverController.x().onTrue(Commands.runOnce(swerve::stopWithX, swerve));
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        swerve.setPose(
                            new Pose2d(swerve.getPose().getTranslation(), new Rotation2d())),
                    swerve)
                .ignoringDisable(true));

    driverController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  currentMode =
                      currentMode == DriveMode.TELEOP ? DriveMode.AUTO_ALIGN : DriveMode.TELEOP;
                },
                shooter,
                swerve));

    operatorController.rightBumper().whileTrue(TeleopCommands.runIntake(intake, shooter));

    operatorController.leftBumper().whileTrue(TeleopCommands.runOuttake(intake, shooter));

    operatorController.povLeft().onTrue(TeleopCommands.setShooterAmpPosition(shooter, hood));

    operatorController.povRight().onTrue(TeleopCommands.setShooterIntakePosition(shooter, hood));

    operatorController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (TeleopCommands.readyToShoot && TeleopCommands.ampMode) {
                    TeleopCommands.runAmpSequence(shooter);
                  } else if (TeleopCommands.readyToShoot) {
                    TeleopCommands.runShootSequence(shooter);
                  } else {
                    Commands.none();
                  }
                },
                shooter));

    operatorController.povUp().onTrue(TeleopCommands.setShooterShootPosition(shooter, hood));
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
    Logger.recordOutput("TeleopCommands/Ready", TeleopCommands.readyToShoot);
    Logger.recordOutput("TeleopCommands/Amp", TeleopCommands.ampMode);
  }
}
