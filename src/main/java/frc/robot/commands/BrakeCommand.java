package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class BrakeCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;

    public BrakeCommand(SwerveSubsystem subsystem) {
        this.swerveSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.brake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}