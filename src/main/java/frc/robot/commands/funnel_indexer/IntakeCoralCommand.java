package frc.robot.commands.funnel_indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FunnelIndexerConstants;
import frc.robot.subsystems.FunnelSubsystem;

import java.util.Map;

public class IntakeCoralCommand extends Command {
    private final FunnelSubsystem m_funnelIndexerSubsystem;

    // Enum representing the state of the beam breaks
    enum BeamBreakState {
        NONE_BROKEN,
        SHALLOW_BROKEN,
        BOTH_BROKEN,
        DEEP_BROKEN
    }

    // Map of beam break states to indexer speeds when we do not have coral
    private static final Map<BeamBreakState, Double> SPEED_MAP = Map.of(
            BeamBreakState.NONE_BROKEN, FunnelIndexerConstants.FULL_SPEED,
            BeamBreakState.SHALLOW_BROKEN, FunnelIndexerConstants.HALF_SPEED,
            BeamBreakState.BOTH_BROKEN, FunnelIndexerConstants.STOP_SPEED,
            BeamBreakState.DEEP_BROKEN, FunnelIndexerConstants.REVERSE_SPEED
    );

    // Map of beam break states to indexer speeds when we do have coral
    private static final Map<BeamBreakState, Double> HAS_CORAL_SPEED_MAP = Map.of(
            BeamBreakState.NONE_BROKEN, FunnelIndexerConstants.STOP_SPEED,
            BeamBreakState.SHALLOW_BROKEN, FunnelIndexerConstants.STOP_SPEED,
            BeamBreakState.BOTH_BROKEN, FunnelIndexerConstants.REVERSE_SPEED,
            BeamBreakState.DEEP_BROKEN, FunnelIndexerConstants.REVERSE_SPEED
    );

    /**
     * Creates a new CorrectCoralPositionCommand.
     */
    public IntakeCoralCommand(FunnelSubsystem funnelIndexerSubsystem) {
        this.m_funnelIndexerSubsystem = funnelIndexerSubsystem;
        addRequirements(funnelIndexerSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    /**
     * Returns the current state of the beam breaks.
     *
     * @return An enum representing which beam breaks are broken.
     */
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
        // Determine the current state of the beam breaks
        BeamBreakState state = getBeamBreakState();

        // Get the speed based on whether we have coral or not
        double speed;
        if (m_funnelIndexerSubsystem.getHasCoral()) {
            speed = HAS_CORAL_SPEED_MAP.get(state);
        } else {
            speed = SPEED_MAP.get(state);
        }

        // Set the indexer speed
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