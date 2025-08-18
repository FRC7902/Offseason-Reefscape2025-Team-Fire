// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class ArmSubsystem extends SubsystemBase {

  /** Main arm TalonFX motor */
  private final TalonFX m_armMotor = new TalonFX(ArmConstants.kArmMotorCANID); 

  /** Motor sim object */
  private final TalonFXSimState m_armMotorSim = m_armMotor.getSimState();

  /** Motor config object */
  private final TalonFXConfiguration m_armMotorConfig = new TalonFXConfiguration();

  /** Arm motor motion magic request object */
  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  /** Arm simulation object */
  private final SingleJointedArmSim m_armSim;

  /** Arm simulation ligament */
  private final MechanismLigament2d m_armLigament = RobotContainer.m_elevatorSubsystem.getMechanism2d().getRoot(
    "ElevatorCarriage", 0, 0
  ).append(new MechanismLigament2d(
      "Arm", 
      ArmConstants.kArmLength, 
      266
    )
  );

  public ArmSubsystem() {

    // Configure motor
    Slot0Configs configs = m_armMotorConfig.Slot0;
    configs.kP = ArmConstants.kArmP;
    configs.kI = ArmConstants.kArmI;
    configs.kD = ArmConstants.kArmD;

    CurrentLimitsConfigs currentLimits = m_armMotorConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = ArmConstants.kStatorCurrentLimit;
    currentLimits.SupplyCurrentLimit = ArmConstants.kSupplyCurrentLimit;

    m_armMotorConfig.Feedback.SensorToMechanismRatio = ArmConstants.kArmGearing;

    m_armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_armMotor.getConfigurator().apply(m_armMotorConfig);

    // Initialize arm simulation
    m_armSim = new SingleJointedArmSim(
      DCMotor.getKrakenX60(1),
      ArmConstants.kArmGearing,
      SingleJointedArmSim.estimateMOI(ArmConstants.kArmLength, ArmConstants.kArmMass),
      ArmConstants.kArmLength,
      Units.degreesToRadians(ArmConstants.kArmMinAngle),
      Units.degreesToRadians(ArmConstants.kArmMaxAngle),
      true,
      Units.degreesToRadians(0)
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Position (deg)", getArmPositionDegrees());
    SmartDashboard.putNumber("Arm position setpoint (rotations)", m_armMotor.getClosedLoopReference().getValueAsDouble());
    updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_armMotorSim.setSupplyVoltage(12);

    // Feed motor voltage into WPILib physics sim
    m_armSim.setInput(m_armMotorSim.getMotorVoltage());
    m_armSim.update(0.02);  

    // Convert mechanism angle to rotor rotations
    double rotorPosition = (m_armSim.getAngleRads() / (2.0 * Math.PI)) * ArmConstants.kArmGearing;
    double rotorVelocity = (m_armSim.getVelocityRadPerSec() / (2.0 * Math.PI)) * ArmConstants.kArmGearing;

    m_armMotorSim.setRawRotorPosition(rotorPosition);
    m_armMotorSim.setRotorVelocity(rotorVelocity);
  }

  /**
   * Updates the telemetry for the arm subsystem.
   */
  public void updateTelemetry() {
    m_armLigament.setAngle(getArmPositionDegrees());
  }

  /**
   * Sets the arm position in degrees.
   * 
   * @param position The desired arm position in degrees.
   */
  public void setArmPositionDegrees(double position) {
    // Ensure position is within bounds
    
    if (position < ArmConstants.kArmMinAngle) {
      position = ArmConstants.kArmMinAngle;
    } else if (position > ArmConstants.kArmMaxAngle) {
      position = ArmConstants.kArmMaxAngle;
    }

    double targetPositionRot = (position / 360.0); // Convert degrees to rotations

    // Set the motion magic request
    m_motionMagicRequest.Position = targetPositionRot;
    m_motionMagicRequest.Slot = 0;
    m_armMotor.setControl(m_motionMagicRequest);
  }

  /**
   * Gets the current arm position in degrees.
   * 
   * @return The current arm position in degrees.
   */
  public double getArmPositionDegrees() {
    double positionDeg = (m_armMotor.getPosition().getValueAsDouble() * 360);
    return positionDeg;
  }

  /**
   * Sets the arm position using an enum for predefined positions.
   * 
   * @param position The desired arm position as an enum.
   */
  public void setArmPositionEnum(ElevatorPosition position) {
    double degrees = switch (position) {
      case ZERO -> ArmConstants.kArmZeroAngle;
      case CORAL_L1 -> ArmConstants.kArmCoralLevel1Angle;
      case CORAL_L2 -> ArmConstants.kArmCoralLevel2Angle;
      case CORAL_L3 -> ArmConstants.kArmCoralLevel3Angle;
      case CORAL_L4 -> ArmConstants.kArmCoralLevel4Angle;
      case PROCESSOR -> ArmConstants.kArmProcessorAngle;
      case ALGAE_HIGH -> ArmConstants.kArmAlgaeHighAngle;
      case ALGAE_LOW -> ArmConstants.kArmAlgaeLowAngle;
      case BARGE -> ArmConstants.kArmBargeAngle;
      default -> ArmConstants.kArmMinAngle;
    };
    setArmPositionDegrees(degrees);
  }

  /**
   * Gets the current arm position as an enum based on predefined angles.
   * 
   * @return The current arm position as an ElevatorPosition enum.
   */
  public ElevatorPosition getArmPositionEnum() {
    double positionDeg = getArmPositionDegrees();
    if (positionDeg == ArmConstants.kArmZeroAngle) {
      return ElevatorPosition.ZERO;
    } else if (positionDeg == ArmConstants.kArmCoralLevel1Angle) {
      return ElevatorPosition.CORAL_L1;
    } else if (positionDeg == ArmConstants.kArmCoralLevel2Angle) {
      return ElevatorPosition.CORAL_L2;
    } else if (positionDeg == ArmConstants.kArmCoralLevel3Angle) {
      return ElevatorPosition.CORAL_L3;
    } else if (positionDeg == ArmConstants.kArmCoralLevel4Angle) {
      return ElevatorPosition.CORAL_L4;
    } else if (positionDeg == ArmConstants.kArmProcessorAngle) {
      return ElevatorPosition.PROCESSOR;
    } else if (positionDeg == ArmConstants.kArmAlgaeHighAngle) {
      return ElevatorPosition.ALGAE_HIGH;
    } else if (positionDeg == ArmConstants.kArmAlgaeLowAngle) {
      return ElevatorPosition.ALGAE_LOW;
    } else if (positionDeg == ArmConstants.kArmBargeAngle) {
      return ElevatorPosition.BARGE;
    }
    return ElevatorPosition.UNKNOWN; // Default case
  }
}
