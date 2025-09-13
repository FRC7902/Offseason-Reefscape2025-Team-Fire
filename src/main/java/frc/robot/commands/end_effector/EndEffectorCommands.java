package frc.robot.commands.end_effector;

import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorCommands {
    public static Intake IntakeEffector(EndEffectorSubsystem endEffectorSubsystem) {
        return new Intake(endEffectorSubsystem);
    }

    public static Outtake OuttakeEffector(EndEffectorSubsystem endEffectorSubsystem) {
        return new Outtake(endEffectorSubsystem);
    }
}
