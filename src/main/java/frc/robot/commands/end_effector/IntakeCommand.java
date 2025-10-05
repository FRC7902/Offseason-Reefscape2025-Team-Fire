// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.end_effector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {

    public enum IntakeMode {
        ALGAE,
        CORAL
    }

    private IntakeMode m_mode;

    /**
     * Creates a new IntakeAlgaeCoralCommand.
     */
    public IntakeCommand(IntakeMode mode) {
        addRequirements(RobotContainer.m_endEffectorSubsystem);

        m_mode = mode;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.m_endEffectorSubsystem.setSpeed(EndEffectorConstants.INTAKE_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Applies a continuous holding speed to the game piece until the OuttakeCommand
        // is run, which then the motors will stop
        if (m_mode == IntakeMode.CORAL) {
            RobotContainer.m_endEffectorSubsystem.setHasCoral(true);
        } else if (RobotContainer.m_endEffectorSubsystem.hasAlgae()){
            RobotContainer.m_endEffectorSubsystem.setSpeed(EndEffectorConstants.ALGAE_HOLD_SPEED);
        } else {
            RobotContainer.m_endEffectorSubsystem.stop();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Coral mode
        if (m_mode == IntakeMode.CORAL) {
            return RobotContainer.m_endEffectorSubsystem.isCoralBeamBreakBroken();
        }

        // Algae mode
        return RobotContainer.m_endEffectorSubsystem.hasAlgae()
                || ((RobotContainer.m_elevatorSubsystem.getElevatorPositionEnum() != ElevatorSubsystem.ElevatorPosition.ALGAE_HIGH)
                && (RobotContainer.m_elevatorSubsystem.getElevatorPositionEnum() != ElevatorSubsystem.ElevatorPosition.ALGAE_LOW));
    }
}
