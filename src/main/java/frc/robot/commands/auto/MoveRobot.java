package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.*;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
public class MoveRobot extends Command {
    public MoveRobot() {
        addRequirements(RobotContainer.m_swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Translation2d moveRobot = new Translation2d(0, 0.5);
        RobotContainer.m_swerveSubsystem.drive(moveRobot, 0, false);
        System.out.println("AAAAAA"
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
