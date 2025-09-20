package frc.robot.commands;

import frc.robot.commands.swerve.StrafeLeftCommand;
import frc.robot.commands.swerve.StrafeRightCommand;

public class SwereCommands {
    public static StrafeLeftCommand StrafeLeft() {
        return new StrafeLeftCommand();
    }

    public static StrafeRightCommand StrafeRight() {
        return new StrafeRightCommand();
    }
}
