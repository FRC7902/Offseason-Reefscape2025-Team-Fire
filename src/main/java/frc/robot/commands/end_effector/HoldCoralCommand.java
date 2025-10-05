package frc.robot.commands.end_effector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class HoldCoralCommand extends Command {

    public HoldCoralCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(RobotContainer.m_endEffectorSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        // Don't do anything if holding algae (not coral)
        if (RobotContainer.m_endEffectorSubsystem.hasAlgae()) {
            return;
        }

        if (RobotContainer.m_endEffectorSubsystem.hasCoral()) {
            if (!RobotContainer.m_endEffectorSubsystem.isCoralBeamBreakBroken()) {
                RobotContainer.m_endEffectorSubsystem.setSpeed(Constants.EndEffectorConstants.CORAL_HOLD_SPEED);
            } else {
                RobotContainer.m_endEffectorSubsystem.stop();
            }
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_endEffectorSubsystem.stop();
    }
}
