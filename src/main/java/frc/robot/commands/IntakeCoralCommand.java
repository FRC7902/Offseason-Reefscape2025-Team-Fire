// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeCoralIndexerConstants;
import frc.robot.subsystems.AlgaeCoralIndexerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoralCommand extends Command {

  private final AlgaeCoralIndexerSubsystem m_algaeCoralIndexerSubsystem;

  /** Creates a new IntakeCoralCommand. */
  public IntakeCoralCommand(AlgaeCoralIndexerSubsystem algaeCoralIndexerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeCoralIndexerSubsystem);
    m_algaeCoralIndexerSubsystem = algaeCoralIndexerSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_algaeCoralIndexerSubsystem.setMotorVoltage(AlgaeCoralIndexerConstants.take_coral);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algaeCoralIndexerSubsystem.setMotorVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_algaeCoralIndexerSubsystem.hasCoral();
  }
}