package frc.robot.commands;

import frc.robot.commands.funnel_indexer.IntakeCoralCommand;
import frc.robot.commands.funnel_indexer.OuttakeCoralCommand;
import frc.robot.subsystems.FunnelSubsystem;

public class FunnelCommands {
    public static OuttakeCoralCommand OuttakeCoral(FunnelSubsystem funnelIndexerSubsystem) {
        return new OuttakeCoralCommand(funnelIndexerSubsystem);
    }

    public static IntakeCoralCommand IntakeCoral(FunnelSubsystem funnelIndexerSubsystem) {
        return new IntakeCoralCommand(funnelIndexerSubsystem);
    }
}
