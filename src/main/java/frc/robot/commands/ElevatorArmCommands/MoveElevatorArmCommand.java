// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorArmCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorArmCommand extends Command {
    boolean isEnum;
    ElevatorSubsystem.ElevatorPosition position;
    double positionMeters;
    double angleDegrees;

    public MoveElevatorArmCommand(ElevatorSubsystem.ElevatorPosition position) {
        isEnum = true;
        this.position = position;
        addRequirements(RobotContainer.m_elevatorSubsystem, RobotContainer.m_armSubsystem);
    }

    public MoveElevatorArmCommand(double positionMeters, double angleDegrees) {
        isEnum = false;
        this.positionMeters = positionMeters;
        this.angleDegrees = angleDegrees;
        addRequirements(RobotContainer.m_elevatorSubsystem, RobotContainer.m_armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (isEnum) {
            RobotContainer.m_elevatorSubsystem.setElevatorPositionEnum(position);
            RobotContainer.m_armSubsystem.setArmPositionEnum(position);
        } else {
            RobotContainer.m_elevatorSubsystem.setElevatorPositionMeters(positionMeters);
            RobotContainer.m_armSubsystem.setArmPositionDegrees(angleDegrees);
        }

//        if (RobotContainer.m_elevatorSubsystem.getElevatorPositionMeters() > Constants.ElevatorConstants.kElevatorCoralLevel1Height) {
//            RobotContainer.m_swerveSubsystem.setSlewRateLimiters(
//                    new SlewRateLimiter(
//                            0.1,
//                            -0.1,
//                            0.0
//                    )
//            );
//        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
//        return RobotContainer.m_elevatorSubsystem.hasReachedSetpoint() && RobotContainer.m_armSubsystem.hasReachedAngle();
        return false;
    }
}
