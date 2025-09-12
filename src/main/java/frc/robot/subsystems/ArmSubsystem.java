// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
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

  /** WCP Through-Bore encoder object */
  private final CANcoder m_armEncoder = new CANcoder(ArmConstants.kArmEncoderPort);

  /** Arm encoder sim state */
  private final CANcoderSimState m_armEncoderSim = m_armEncoder.getSimState();

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
  /** Motor voltage request object */
  private VoltageOut m_voltageRequest = new VoltageOut(0);

  /** SysId routine object to determine S, V, and A constants */
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          Volts.of(4),
          null,
          (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
          (volts) -> m_armMotor.setControl(m_voltageRequest.withOutput(volts.in(Volts))),
          null,
          this
      )
  );

  /** Arm simulation ligament */
  private final MechanismLigament2d m_coralIndexerLigament = m_armLigament.append(
    new MechanismLigament2d(
      "CoralIndexer", 
      Units.inchesToMeters(13),
      (-180) + 62.552
    )
  );

  public ArmSubsystem() {
    if (ArmConstants.kTuningMode) {
      SmartDashboard.putNumber("Arm P", ArmConstants.kArmP);
      SmartDashboard.putNumber("Arm I", ArmConstants.kArmI);
      SmartDashboard.putNumber("Arm D", ArmConstants.kArmD);
      SmartDashboard.putNumber("Arm G", ArmConstants.kArmG);
    }

    // Configure motor
    Slot0Configs configs = m_armMotorConfig.Slot0;
    configs.kP = ArmConstants.kArmP;
    configs.kI = ArmConstants.kArmI;
    configs.kD = ArmConstants.kArmD;
    configs.kS = ArmConstants.kArmS;
    configs.kV = ArmConstants.kArmV;
    configs.kG = ArmConstants.kArmG;
    configs.kA = ArmConstants.kArmA;
    configs.GravityType = GravityTypeValue.Arm_Cosine;

    CurrentLimitsConfigs currentLimits = m_armMotorConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = ArmConstants.kStatorCurrentLimit;
    currentLimits.SupplyCurrentLimit = ArmConstants.kSupplyCurrentLimit;

    m_armMotorConfig.MotionMagic.MotionMagicAcceleration = 20;
    m_armMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 20;

    m_armMotorConfig.Feedback.SensorToMechanismRatio = 1;

    m_armMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    m_armMotorConfig.Feedback.FeedbackRemoteSensorID = m_armEncoder.getDeviceID();

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
    if (ArmConstants.kTuningMode) {
      // Track previous and current values of constants
      double previousP = ArmConstants.kArmP;
      double previousI = ArmConstants.kArmI;
      double previousD = ArmConstants.kArmD;
      double previousG = ArmConstants.kArmG;

      ArmConstants.kArmP = SmartDashboard.getNumber("Arm P", ArmConstants.kArmP);
      ArmConstants.kArmI = SmartDashboard.getNumber("Arm I", ArmConstants.kArmI);
      ArmConstants.kArmD = SmartDashboard.getNumber("Arm D", ArmConstants.kArmD);
      ArmConstants.kArmG = SmartDashboard.getNumber("Arm G", ArmConstants.kArmG);

      // Reapply config if any constants have changed
      if (previousP != ArmConstants.kArmP || previousI != ArmConstants.kArmI || previousD != ArmConstants.kArmD || previousG != ArmConstants.kArmG) {
        m_armMotorConfig.Slot0.kP = ElevatorConstants.kElevatorP;
        m_armMotorConfig.Slot0.kI = ElevatorConstants.kElevatorI;
        m_armMotorConfig.Slot0.kD = ElevatorConstants.kElevatorD;
        m_armMotorConfig.Slot0.kG = ElevatorConstants.kElevatorG;
        m_armMotor.getConfigurator().apply(m_armMotorConfig);
      }
    }

    SmartDashboard.putNumber("Arm Position (deg)", getArmPositionDegrees()/360);
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
    m_armEncoderSim.setRawPosition(rotorPosition/ArmConstants.kArmGearing); // Divide to account for gearing
    m_armMotorSim.setRotorVelocity(rotorVelocity);
  }

  /**
   * Updates the telemetry for the arm subsystem.
   */
  public void updateTelemetry() {
    m_armLigament.setAngle(m_armSim.getAngleRads() * (180 / Math.PI)); // Convert radians to degrees
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

    if ((getElevatorPositionMeters() < Units.inchesToMeters(34) || getElevatorSetpointMeters() < Units.inchesToMeters(34)) && position > 50) {
      position = 50;
    } else if ((getElevatorPositionMeters() > Units.inchesToMeters(34) || getElevatorSetpointMeters() > Units.inchesToMeters(34)) && position < -45) {
      position = -45;
    }
    
    double targetPositionRot = (position / 360.0) ; // Convert degrees to rotations

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
    double positionDeg = (m_armEncoder.getPosition().getValueAsDouble() * 360);
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

  /**
   * Gets the current arm setpoint in degrees.
   * 
   * @return The current arm setpoint in degrees.
   */
  public double getArmSetpointDegrees() {
    return m_armMotor.getClosedLoopReference().getValueAsDouble();
  }

  /**
   * Gets the current elevator setpoint in meters
   */
  public double getElevatorSetpointMeters() {
    return RobotContainer.m_elevatorSubsystem.getSetpoint();
  }

  /**
   * Get the elevator position in meters
   * 
   * @return The elevator position in meters
   */
  private double getElevatorPositionMeters() {
    return RobotContainer.m_elevatorSubsystem.getElevatorPositionMeters();
  }

  /**
   * SysId quasistatic test
   * 
   * @param direction The direction of the test (up or down)
   * @see SysIdRoutine.Direction
   */
  public void sysIdQuasistatic(SysIdRoutine.Direction direction) {
      m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * SysId dynamic test
   * 
   * @param direction The direction of the test (up or down)
   * @see SysIdRoutine.Direction
   */
  public void sysIdDynamic(SysIdRoutine.Direction direction) {
      m_sysIdRoutine.dynamic(direction);
  }
}