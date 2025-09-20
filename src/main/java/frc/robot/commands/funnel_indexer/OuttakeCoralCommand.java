package frc.robot.commands.funnel_indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FunnelIndexerConstants;
import frc.robot.subsystems.FunnelSubsystem;

public class OuttakeCoralCommand extends Command {
    private final FunnelSubsystem m_funnelIndexerSubsystem;

    public OuttakeCoralCommand(FunnelSubsystem funnelIndexerSubsystem) {
        this.m_funnelIndexerSubsystem = funnelIndexerSubsystem;
        addRequirements(funnelIndexerSubsystem);
    }

    @Override
    public void execute() {
        m_funnelIndexerSubsystem.setIndexerSpeed(FunnelIndexerConstants.OUTTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return !m_funnelIndexerSubsystem.isShallowBeamBreakBroken() && !m_funnelIndexerSubsystem.isDeepBeamBreakBroken();
    }
}