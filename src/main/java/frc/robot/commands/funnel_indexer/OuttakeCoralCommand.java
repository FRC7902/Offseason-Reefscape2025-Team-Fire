package frc.robot.commands.funnel_indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FunnelIndexerConstants;
import frc.robot.RobotContainer;

public class OuttakeCoralCommand extends Command {

    public OuttakeCoralCommand() {
        addRequirements(RobotContainer.m_funnelIndexerSubsystem);
    }

    @Override
    public void execute() {
        RobotContainer.m_funnelIndexerSubsystem.setIndexerSpeed(FunnelIndexerConstants.OUTTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_funnelIndexerSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return !RobotContainer.m_funnelIndexerSubsystem.isShallowBeamBreakBroken() && !RobotContainer.m_funnelIndexerSubsystem.isDeepBeamBreakBroken();
    }
}