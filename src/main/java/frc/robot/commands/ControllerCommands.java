package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.shooter.Shooter;
public class ControllerCommands extends Command{
    private Flywheel s_Flywheel; 
    private Shooter s_Shooter; 
    private Supplier<CurrentState> currentState; 
    
}
