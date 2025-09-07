// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffectorCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeCoralIndexerConstants;
import frc.robot.commands.ElevatorArmCommands.MoveElevatorArmCommand;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.vision.PhotonSubsystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorArmCommands.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgaeCoralCommand extends Command {
  MoveElevatorArmCommand moveElvArmCom;
  /** Creates a new IntakeAlgaeCoralCommand. */
  public IntakeAlgaeCoralCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(
      RobotContainer.m_endEffectorSubsystem,
      RobotContainer.m_elevatorSubsystem,
      RobotContainer.m_photonSubsystem
    );
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // All elv logic
    Boolean height = false;
    if (!RobotBase.isSimulation()) {
      int currentTag = RobotContainer.m_photonSubsystem.getTagID();
      for (int i : PhotonSubsystem.reefIDHeights.keySet()){
        if (currentTag == i) {
          height = PhotonSubsystem.reefIDHeights.get(i);
          moveElvArmCom = height ? new MoveElevatorArmCommand(ElevatorPosition.ALGAE_HIGH) : new MoveElevatorArmCommand(ElevatorPosition.ALGAE_LOW);
        }
        
      }
      andThen(moveElvArmCom);
    }
    RobotContainer.m_endEffectorSubsystem.setSpeed(AlgaeCoralIndexerConstants.kIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_endEffectorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_endEffectorSubsystem.isCoralDetected() || RobotContainer.m_endEffectorSubsystem.isAlgaeDetected();
  }
}
