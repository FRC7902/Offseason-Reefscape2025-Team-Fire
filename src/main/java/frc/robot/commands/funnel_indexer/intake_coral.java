// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.funnel_indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.End_effector_subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class intake_coral extends Command {
  private final End_effector_subsystem m_end_effector_subsystem = RobotContainer.m_endEffectorSubsystem;
  /** Creates a new intake_coral. */
  public intake_coral() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_end_effector_subsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_end_effector_subsystem.setMotorSpeed(0.5);  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_end_effector_subsystem.setMotorSpeed(0);  
  // Returns true when the command should end.
  }
  @Override
  public boolean isFinished() {
    if(m_end_effector_subsystem.getCoralBeamBreak()==true){
      return true;
    }
    return false;
  }
}
