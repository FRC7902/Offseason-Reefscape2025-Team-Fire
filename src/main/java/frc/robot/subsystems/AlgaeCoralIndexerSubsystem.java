// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeCoralIndexerConstants;

public class AlgaeCoralIndexerSubsystem extends SubsystemBase {

  private final TalonFX m_motor;

  private DigitalInput m_algaeBeamBreak;
  private DigitalInput m_coralBeamBreak;

  /** Creates a new AlgaeCoralIndexerSubsystem. */
  public AlgaeCoralIndexerSubsystem() {
    m_motor = new TalonFX(AlgaeCoralIndexerConstants.ALGAE_CORAL_INDEXER_MOTOR_ID);

    m_algaeBeamBreak = new DigitalInput(AlgaeCoralIndexerConstants.ALGAE_BEAM_BREAK_ID);
    m_coralBeamBreak = new DigitalInput(AlgaeCoralIndexerConstants.CORAL_BEAM_BREAK_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorSpeed(double speed) {
    m_motor.set(speed);
  }

  public void setMotorVoltage(double power) {
    m_motor.setVoltage(power);
  }

  public boolean hasAlgae() {
    return !m_algaeBeamBreak.get();
  }

  public boolean hasCoral() {
    return !m_coralBeamBreak.get();
  }
}
