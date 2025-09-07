package frc.robot.commands.funnel_indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FunnelIndexerConstants;
import frc.robot.subsystems.FunnelIndexerSubsystem;

import java.util.Map;

public class IntakeCoral extends Command {
    private final FunnelIndexerSubsystem m_funnelIndexerSubsystem;

    enum BeamBreakState {
        NONE_BROKEN,
        SHALLOW_BROKEN,
        BOTH_BROKEN,
        DEEP_BROKEN
    }

    private static final Map<BeamBreakState, Double> SPEED_MAP = Map.of(
            BeamBreakState.NONE_BROKEN, FunnelIndexerConstants.FULL_SPEED,
            BeamBreakState.SHALLOW_BROKEN, FunnelIndexerConstants.HALF_SPEED,
            BeamBreakState.BOTH_BROKEN, FunnelIndexerConstants.STOP_SPEED,
            BeamBreakState.DEEP_BROKEN, FunnelIndexerConstants.REVERSE_SPEED
    );

    private static final Map<BeamBreakState, Double> HAS_CORAL_SPEED_MAP = Map.of(
            BeamBreakState.NONE_BROKEN, FunnelIndexerConstants.STOP_SPEED,
            BeamBreakState.SHALLOW_BROKEN, FunnelIndexerConstants.STOP_SPEED,
            BeamBreakState.BOTH_BROKEN, FunnelIndexerConstants.REVERSE_SPEED,
            BeamBreakState.DEEP_BROKEN, FunnelIndexerConstants.REVERSE_SPEED
    );

    /**
     * Creates a new CorrectCoralPositionCommand.
     */
    public IntakeCoral(FunnelIndexerSubsystem funnelIndexerSubsystem) {
        this.m_funnelIndexerSubsystem = funnelIndexerSubsystem;
        addRequirements(funnelIndexerSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    private BeamBreakState getBeamBreakState() {
        boolean shallow = m_funnelIndexerSubsystem.isShallowBeamBreakBroken();
        boolean deep = m_funnelIndexerSubsystem.isDeepBeamBreakBroken();
        if (!shallow && !deep) return BeamBreakState.NONE_BROKEN;
        if (shallow && deep) return BeamBreakState.BOTH_BROKEN;

        // Deep is always false here
        if (shallow) return BeamBreakState.SHALLOW_BROKEN;
        return BeamBreakState.DEEP_BROKEN;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        BeamBreakState state = getBeamBreakState();
        double speed;
        if (m_funnelIndexerSubsystem.getHasCoral()) {
            speed = HAS_CORAL_SPEED_MAP.get(state);
        } else {
            speed = SPEED_MAP.get(state);
        }
        m_funnelIndexerSubsystem.setIndexerSpeed(speed);
    }

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