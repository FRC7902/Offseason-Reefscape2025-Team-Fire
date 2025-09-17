package frc.robot.commands;

import frc.robot.commands.end_effector.IntakeCommand;
import frc.robot.commands.end_effector.IntakeCommand.IntakeMode;
import frc.robot.commands.end_effector.OuttakeCommand;

public class EndEffectorCommands {
    public static IntakeCommand IntakeEffector(IntakeMode intakeMode) {
        return new IntakeCommand(intakeMode);
    }

    public static OuttakeCommand OuttakeEffector() {
        return new OuttakeCommand();
    }
}
