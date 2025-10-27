package frc.robot.commands;

import frc.robot.commands.vision.AutoAlignToReef;

public class AutoAlignCommands {
    public static AutoAlignToReef AutoAlignLeft() {
        return new AutoAlignToReef(AutoAlignToReef.ReefBranchSide.LEFT);
    }

    public static AutoAlignToReef AutoAlignRight() {
        return new AutoAlignToReef(AutoAlignToReef.ReefBranchSide.RIGHT);
    }

    public static AutoAlignToReef AutoAlignCenter() {
        return new AutoAlignToReef(AutoAlignToReef.ReefBranchSide.CENTER);
    }
}
