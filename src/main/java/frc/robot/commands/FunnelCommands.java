package frc.robot.commands;

import frc.robot.commands.funnel_indexer.IntakeCoral;
import frc.robot.commands.funnel_indexer.OuttakeCoral;
import frc.robot.subsystems.FunnelSubsystem;

public class FunnelCommands {
    public static OuttakeCoral OuttakeCoral(FunnelSubsystem funnelIndexerSubsystem) {
        return new OuttakeCoral(funnelIndexerSubsystem);
    }

    public static IntakeCoral IntakeCoral(FunnelSubsystem funnelIndexerSubsystem) {
        return new IntakeCoral(funnelIndexerSubsystem);
    }
}
