package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class LockPoseCommand extends Command {
    private final SwerveSubsystem swerve;
    
    public LockPoseCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {
        swerve.brake();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}