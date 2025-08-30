package frc.robot.commands.funnel_indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FunnelIndexerConstants;
import frc.robot.subsystems.FunnelIndexerSubsystem;

public class OuttakeCoral extends Command {
  private final FunnelIndexerSubsystem m_funnelIndexerSubsystem;

  public OuttakeCoral(FunnelIndexerSubsystem funnelIndexerSubsystem) {
    this.m_funnelIndexerSubsystem = funnelIndexerSubsystem;
    addRequirements(funnelIndexerSubsystem);
  }

  @Override
  public void execute() {
    m_funnelIndexerSubsystem.setIndexerSpeed(FunnelIndexerConstants.HALF_SPEED);
  }

  @Override
  public boolean isFinished() {
    return m_funnelIndexerSubsystem.isShallowBeamBroken() && m_funnelIndexerSubsystem.isDeepBeamBroken();
  }
}