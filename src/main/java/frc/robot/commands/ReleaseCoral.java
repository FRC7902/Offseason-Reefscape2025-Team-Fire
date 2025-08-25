package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FunnelIndexerSubsystem;


/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#
 * defining-commands
 */
// ...existing code...
public class ReleaseCoral extends Command {
  private final FunnelIndexerSubsystem funnelIndexerSubsystem;

  /** Creates a new ReleaseCoral command. */
  public ReleaseCoral(FunnelIndexerSubsystem funnelIndexerSubsystem) {
    this.funnelIndexerSubsystem = funnelIndexerSubsystem;
    addRequirements(funnelIndexerSubsystem);
  }

  @Override
  public void execute() {
    funnelIndexerSubsystem.setPower(Constants.FunnelIndexerConstants.m_halfSpeed);
  }

  @Override
  public boolean isFinished() {
    return funnelIndexerSubsystem.hasCoral();
  }
}
// ...existing code...