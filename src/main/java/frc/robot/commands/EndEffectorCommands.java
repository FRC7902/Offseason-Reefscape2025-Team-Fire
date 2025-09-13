package frc.robot.commands;

import frc.robot.commands.end_effector.IntakeCommand;
import frc.robot.commands.end_effector.OuttakeCommand;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorCommands {
    public static IntakeCommand IntakeEffector(EndEffectorSubsystem endEffectorSubsystem) {
        return new IntakeCommand(endEffectorSubsystem);
    }

    public static OuttakeCommand OuttakeEffector(EndEffectorSubsystem endEffectorSubsystem) {
        return new OuttakeCommand(endEffectorSubsystem);
    }
}
