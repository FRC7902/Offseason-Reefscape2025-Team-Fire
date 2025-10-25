package frc.robot.commands;

import frc.robot.commands.end_effector.HoldCoralCommand;
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

    public static OuttakeCommand OuttakeEffector(double outtakeSpeed) {
        return new OuttakeCommand(outtakeSpeed);
    }

    public static HoldCoralCommand HoldCoralCommand() {
        return new HoldCoralCommand();
    }
}
