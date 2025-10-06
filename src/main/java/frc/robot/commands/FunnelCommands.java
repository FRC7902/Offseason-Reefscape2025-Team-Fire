package frc.robot.commands;

import frc.robot.commands.funnel_indexer.IntakeCoralCommand;
import frc.robot.commands.funnel_indexer.OuttakeCoralCommand;
import frc.robot.commands.funnel_indexer.PassThroughCoralCommand;

public class FunnelCommands {
    public static OuttakeCoralCommand OuttakeCoral() {
        return new OuttakeCoralCommand();
    }

    public static IntakeCoralCommand IntakeCoral() {
        return new IntakeCoralCommand();
    }

    public static PassThroughCoralCommand PassThroughCoral() {
        return new PassThroughCoralCommand();
    }
}
