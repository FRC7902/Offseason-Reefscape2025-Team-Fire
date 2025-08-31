package frc.robot.commands.funnel_indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FunnelIndexerConstants;
import frc.robot.subsystems.FunnelIndexerSubsystem;

public class IntakeCoral extends Command {
  private final FunnelIndexerSubsystem m_funnelIndexerSubsystem;

  /** Creates a new CorrectCoralPositionCommand. */
  public IntakeCoral(FunnelIndexerSubsystem funnelIndexerSubsystem) {
    this.m_funnelIndexerSubsystem = funnelIndexerSubsystem;
    addRequirements(funnelIndexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // RobotContainer.m_operatorController.setRumble(GenericHID.RumbleType.kBothRumble,
    // 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  // ...existing code...
  @Override
  public void execute() {
    if (!m_funnelIndexerSubsystem.isShallowBeamBroken()) {
      m_funnelIndexerSubsystem.setIndexerSpeed(FunnelIndexerConstants.FULL_SPEED);
    } else if (m_funnelIndexerSubsystem.isShallowBeamBroken()) {
      m_funnelIndexerSubsystem.setIndexerSpeed(FunnelIndexerConstants.HALF_SPEED);
    } else if (m_funnelIndexerSubsystem.isShallowBeamBroken()
        && m_funnelIndexerSubsystem.isDeepBeamBroken()) {
      m_funnelIndexerSubsystem.setIndexerSpeed(FunnelIndexerConstants.STOP_SPEED);
    } else if (m_funnelIndexerSubsystem.isDeepBeamBroken()) {
      m_funnelIndexerSubsystem.setIndexerSpeed(FunnelIndexerConstants.REVERSE_SPEED);
    } else {
      m_funnelIndexerSubsystem.setIndexerSpeed(FunnelIndexerConstants.STOP_SPEED);
    }
  }
// ...existing code...

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_funnelIndexerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}