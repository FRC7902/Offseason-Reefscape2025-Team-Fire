package frc.robot.commands.funnel_indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class PassThroughCoralCommand extends Command {

    public PassThroughCoralCommand() {
        addRequirements(RobotContainer.m_funnelIndexerSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.m_funnelIndexerSubsystem.setIndexerSpeed(Constants.FunnelIndexerConstants.FULL_SPEED);
    }

    @Override
    public boolean isFinished() {
        return !RobotContainer.m_funnelIndexerSubsystem.isShallowBeamBreakBroken() && RobotContainer.m_funnelIndexerSubsystem.isDeepBeamBreakBroken();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_funnelIndexerSubsystem.stop();
    }
}
