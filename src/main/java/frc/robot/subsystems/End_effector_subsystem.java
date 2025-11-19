// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EndEffectorConstants;

public class End_effector_subsystem extends SubsystemBase {
    private final TalonFX Endeffectormotor;
    private DigitalInput shallow_beanbreak;
  /** Creates a new End_infector_subsystem. */
    private final TalonFXConfiguration Endeffectormotorconfig;
  public End_effector_subsystem() {
    Endeffectormotor= new TalonFX(EndEffectorConstants.MOTOR_CAN_ID);
    Endeffectormotorconfig= new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimits = Endeffectormotorconfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = EndEffectorConstants.MOTOR_STATOR_CURRENT_LIMIT;
    currentLimits.SupplyCurrentLimit = EndEffectorConstants.MOTOR_SUPPLY_CURRENT_LIMIT;
    Endeffectormotor.getConfigurator().apply(Endeffectormotorconfig);
    shallow_beanbreak = new DigitalInput(EndEffectorConstants.CORAL_BEAM_BREAK_PORT_ID);
  }

  public boolean getCoralBeamBreak(){
    return !shallow_beanbreak.get();
  }
  public void periodic() {
    // This method will be called once per scheduler run
    //smartdashboard
    SmartDashboard.putBoolean("Coral Beam Break", getCoralBeamBreak());
  }
  public void setMotorSpeed(double INTAKE_SPEED) {
    Endeffectormotor.set(INTAKE_SPEED);
  }

}
