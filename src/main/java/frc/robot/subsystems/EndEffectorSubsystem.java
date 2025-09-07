// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {

  private final TalonFX m_motor;
  private final TalonFXConfiguration m_motorConfig;
  private final MotorOutputConfigs m_motorOutputConfig;

  // private final DigitalInput m_coralBeamBreak;
  // private final DigitalInput m_algaeProximitySensor;


  /** Creates a new AlgaeCoralIndexerSubsystem. */
  public EndEffectorSubsystem() {
    m_motor = new TalonFX(EndEffectorConstants.MOTOR_CAN_ID);

    m_motorConfig = new TalonFXConfiguration();
    m_motorOutputConfig = new MotorOutputConfigs();
    
    // m_coralBeamBreak = new DigitalInput(EndEffectorConstants.CORAL_BEAM_BREAK_PORT_ID);
    // m_algaeProximitySensor = new DigitalInput(EndEffectorConstants.ALGAE_PROXIMITY_SENSOR_PORT_ID);
    
    m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    m_motorConfig.CurrentLimits.StatorCurrentLimit = EndEffectorConstants.MOTOR_STATOR_CURRENT_LIMIT;
    
    m_motorOutputConfig.withNeutralMode(NeutralModeValue.Brake);
    m_motorConfig.withMotorOutput(m_motorOutputConfig);

    m_motor.getConfigurator().apply(m_motorConfig);

  }

  public boolean isCoralDetected() {
    // return m_coralBeamBreak.get();
    return false;
  }

  public boolean isAlgaeDetected() {
    // return m_algaeProximitySensor.get();
    return false;
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
