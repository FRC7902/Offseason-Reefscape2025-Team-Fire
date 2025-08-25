package frc.robot.commands;

import frc.robot.commands.funnel_indexer.IntakeCoral;
import frc.robot.commands.funnel_indexer.OuttakeCoral;
import frc.robot.subsystems.FunnelIndexerSubsystem;

public class FunnelIndexerCommands {
  private FunnelIndexerCommands() {}

  public static OuttakeCoral OuttakeCoral(FunnelIndexerSubsystem funnelIndexerSubsystem) {
    return new OuttakeCoral(funnelIndexerSubsystem);
  }

  public static IntakeCoral IntakeCoral(FunnelIndexerSubsystem funnelIndexerSubsystem) {
    return new IntakeCoral(funnelIndexerSubsystem);
  }
}
