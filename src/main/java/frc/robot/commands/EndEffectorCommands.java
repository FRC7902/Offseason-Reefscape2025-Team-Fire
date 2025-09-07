package frc.robot.commands;

import frc.robot.commands.end_effector.IntakeEffectorCommand;
import frc.robot.commands.end_effector.OuttakeEffectorCommand;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorCommands {
    public static IntakeEffectorCommand IntakeEffector(EndEffectorSubsystem endEffectorSubsystem) {
        return new IntakeEffectorCommand(endEffectorSubsystem);
    }

    public static OuttakeEffectorCommand OuttakeEffector(EndEffectorSubsystem endEffectorSubsystem) {
        return new OuttakeEffectorCommand(endEffectorSubsystem);
    }
}
