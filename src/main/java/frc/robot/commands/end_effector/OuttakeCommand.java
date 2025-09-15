// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.end_effector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.EndEffectorSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OuttakeCommand extends Command {


    /**
     * Creates a new OuttakeAlgaeCoralCommand.
     */
    public OuttakeCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_endEffectorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.m_endEffectorSubsystem.setSpeed(EndEffectorConstants.OUTTAKE_SPEED);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_endEffectorSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return !RobotContainer.m_endEffectorSubsystem.isCoralDetected() && !RobotContainer.m_endEffectorSubsystem.isAlgaeDetected();
        return false;
    }
}
