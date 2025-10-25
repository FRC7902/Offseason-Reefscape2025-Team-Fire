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
import com.ctre.phoenix6.signals.InvertedValue;
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

    /**
     * Main arm TalonFX motor
     */
    private final TalonFX m_armMotor = new TalonFX(ArmConstants.MOTOR_CAN_ID);

    /**
     * Motor sim object
     */
    private final TalonFXSimState m_armMotorSim = m_armMotor.getSimState();

    /**
     * Motor config object
     */
    private final TalonFXConfiguration m_armMotorConfig = new TalonFXConfiguration();

    /**
     * Arm motor motion magic request object
     */
    private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    /**
     * WCP Through-Bore encoder object
     */
    private final CANcoder m_armEncoder = new CANcoder(ArmConstants.ENCODER_CAN_ID);

    /**
     * Arm encoder sim state
     */
    private final CANcoderSimState m_armEncoderSim = m_armEncoder.getSimState();

    /**
     * Arm simulation object
     */
    private final SingleJointedArmSim m_armSim;

    /**
     * Arm simulation ligament
     */
    private final MechanismLigament2d m_armLigament = RobotContainer.m_elevatorSubsystem.getMechanism2d().getRoot(
            "ElevatorCarriage", 0, 0).append(
                    new MechanismLigament2d(
                            "Arm",
                            ArmConstants.LENGTH_METERS,
                            266));
    /**
     * Motor voltage request object
     */
    private VoltageOut m_voltageRequest = new VoltageOut(0);

    /**
     * SysId routine object to determine S, V, and A constants
     */
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> m_armMotor.setControl(m_voltageRequest.withOutput(volts.in(Volts))),
                    null,
                    this));

    /**
     * Arm simulation ligament
     */
    private final MechanismLigament2d m_coralIndexerLigament = m_armLigament.append(
            new MechanismLigament2d(
                    "CoralIndexer",
                    Units.inchesToMeters(13),
                    (-180) + 62.552));

    public ArmSubsystem() {
        if (ArmConstants.TUNING_MODE_ENABLED) {
            SmartDashboard.putNumber("arm/tuning/P", ArmConstants.PID_P);
            SmartDashboard.putNumber("arm/tuning/I", ArmConstants.PID_I);
            SmartDashboard.putNumber("arm/tuning/D", ArmConstants.PID_D);
            SmartDashboard.putNumber("arm/tuning/G", ArmConstants.FF_G);

            SmartDashboard.putNumber("arm/tuning/setpoint/Zero", ArmConstants.ZERO_ANGLE_DEGREES);
            SmartDashboard.putNumber("arm/tuning/setpoint/Rest", ArmConstants.REST_ANGLE_DEGREES);
            SmartDashboard.putNumber("arm/tuning/setpoint/Coral L1", ArmConstants.L1_ANGLE_DEGREES);
            SmartDashboard.putNumber("arm/tuning/setpoint/Coral L2", ArmConstants.L2_ANGLE_DEGREES);
            SmartDashboard.putNumber("arm/tuning/setpoint/Coral L3", ArmConstants.L3_ANGLE_DEGREES);
            SmartDashboard.putNumber("arm/tuning/setpoint/Coral L4", ArmConstants.L4_ANGLE_DEGREES);
            SmartDashboard.putNumber("arm/tuning/setpoint/Processor", ArmConstants.PROCESSOR_ANGLE_DEGREES);
            SmartDashboard.putNumber("arm/tuning/setpoint/Algae High", ArmConstants.HIGH_ALGAE_ANGLE_DEGREES);
            SmartDashboard.putNumber("arm/tuning/setpoint/Algae Low", ArmConstants.LOW_ALGAE_ANGLE_DEGREES);
            SmartDashboard.putNumber("arm/tuning/setpoint/Barge", ArmConstants.BARGE_ANGLE_DEGREES);
        }

        // Configure motor
        Slot0Configs configs = m_armMotorConfig.Slot0;
        configs.kP = ArmConstants.PID_P;
        configs.kI = ArmConstants.PID_I;
        configs.kD = ArmConstants.PID_D;
        configs.kS = ArmConstants.FF_S;
        configs.kV = ArmConstants.FF_V;
        configs.kG = ArmConstants.FF_G;
        configs.kA = ArmConstants.FF_A;
        configs.GravityType = GravityTypeValue.Arm_Cosine;

        CurrentLimitsConfigs currentLimits = m_armMotorConfig.CurrentLimits;
        currentLimits.StatorCurrentLimit = ArmConstants.STATOR_CURRENT_LIMIT;
        currentLimits.SupplyCurrentLimit = ArmConstants.SUPPLY_CURRENT_LIMIT;

        m_armMotorConfig.MotionMagic.MotionMagicAcceleration = 20;
        m_armMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 20;

        m_armMotorConfig.Feedback.SensorToMechanismRatio = 1;

        m_armMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        m_armMotorConfig.Feedback.FeedbackRemoteSensorID = m_armEncoder.getDeviceID();

        m_armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_armMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_armMotor.getConfigurator().apply(m_armMotorConfig);

        // Initialize arm simulation
        m_armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                ArmConstants.GEARING,
                SingleJointedArmSim.estimateMOI(ArmConstants.LENGTH_METERS, ArmConstants.MASS_KG),
                ArmConstants.LENGTH_METERS,
                Units.degreesToRadians(ArmConstants.MIN_ANGLE_DEGREES),
                Units.degreesToRadians(ArmConstants.MAX_ANGLE_DEGREES),
                true,
                Units.degreesToRadians(0));
    }

    @Override
    public void periodic() {
        if (ArmConstants.TUNING_MODE_ENABLED) {
            // Track previous and current values of constants
            double previousP = ArmConstants.PID_P;
            double previousI = ArmConstants.PID_I;
            double previousD = ArmConstants.PID_D;
            double previousG = ArmConstants.FF_G;

            ArmConstants.PID_P = SmartDashboard.getNumber("arm/tuning/PID_P", ArmConstants.PID_P);
            ArmConstants.PID_I = SmartDashboard.getNumber("arm/tuning/PID_I", ArmConstants.PID_I);
            ArmConstants.PID_D = SmartDashboard.getNumber("arm/tuning/PID_D", ArmConstants.PID_D);
            ArmConstants.FF_G = SmartDashboard.getNumber("arm/tuning/FF_G", ArmConstants.FF_G);

            ArmConstants.ZERO_ANGLE_DEGREES = SmartDashboard.getNumber("arm/tuning/setpoint/Zero", ArmConstants.ZERO_ANGLE_DEGREES);
            ArmConstants.REST_ANGLE_DEGREES = SmartDashboard.getNumber("arm/tuning/setpoint/Rest", ArmConstants.REST_ANGLE_DEGREES);
            ArmConstants.L1_ANGLE_DEGREES = SmartDashboard.getNumber("arm/tuning/setpoint/Coral L1", ArmConstants.L1_ANGLE_DEGREES);
            ArmConstants.L2_ANGLE_DEGREES = SmartDashboard.getNumber("arm/tuning/setpoint/Coral L2", ArmConstants.L2_ANGLE_DEGREES);
            ArmConstants.L3_ANGLE_DEGREES = SmartDashboard.getNumber("arm/tuning/setpoint/Coral L3", ArmConstants.L3_ANGLE_DEGREES);
            ArmConstants.L4_ANGLE_DEGREES = SmartDashboard.getNumber("arm/tuning/setpoint/Coral L4", ArmConstants.L4_ANGLE_DEGREES);
            ArmConstants.PROCESSOR_ANGLE_DEGREES = SmartDashboard.getNumber("arm/tuning/setpoint/Processor", ArmConstants.PROCESSOR_ANGLE_DEGREES);
            ArmConstants.HIGH_ALGAE_ANGLE_DEGREES = SmartDashboard.getNumber("arm/tuning/setpoint/Algae High", ArmConstants.HIGH_ALGAE_ANGLE_DEGREES);
            ArmConstants.LOW_ALGAE_ANGLE_DEGREES = SmartDashboard.getNumber("arm/tuning/setpoint/Algae Low", ArmConstants.LOW_ALGAE_ANGLE_DEGREES);
            ArmConstants.BARGE_ANGLE_DEGREES = SmartDashboard.getNumber("arm/tuning/setpoint/Barge", ArmConstants.BARGE_ANGLE_DEGREES);

            // Reapply config if any constants have changed
            if (previousP != ArmConstants.PID_P || previousI != ArmConstants.PID_I || previousD != ArmConstants.PID_D
                    || previousG != ArmConstants.FF_G) {
                m_armMotorConfig.Slot0.kP = ElevatorConstants.PID_P;
                m_armMotorConfig.Slot0.kI = ElevatorConstants.PID_I;
                m_armMotorConfig.Slot0.kD = ElevatorConstants.PID_D;
                m_armMotorConfig.Slot0.kG = ElevatorConstants.FF_G;
                m_armMotor.getConfigurator().apply(m_armMotorConfig);
            }
        }

        SmartDashboard.putNumber("arm/info/Position (rotations)", getArmPositionDegrees() / 360);
        SmartDashboard.putNumber("arm/info/position setpoint (rotations)",
                m_armMotor.getClosedLoopReference().getValueAsDouble());

        SmartDashboard.putNumber("arm/info/Position (Degrees)", getArmPositionDegrees());
        SmartDashboard.putNumber("arm/info/Setpoint (Degrees)", getArmSetpointDegrees());

        SmartDashboard.putBoolean("arm/info/Reached Angle", hasReachedAngle());
        SmartDashboard.putString("arm/info/Position Enum", getArmPositionEnum().toString());

        updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_armMotorSim.setSupplyVoltage(12);

        // Feed motor voltage into WPILib physics sim
        m_armSim.setInput(m_armMotorSim.getMotorVoltage());
        m_armSim.update(0.02);

        // Convert mechanism angle to rotor rotations

        double rotorPosition = (m_armSim.getAngleRads() / (2.0 * Math.PI)) * ArmConstants.GEARING;
        double rotorVelocity = (m_armSim.getVelocityRadPerSec() / (2.0 * Math.PI)) * ArmConstants.GEARING;

        m_armMotorSim.setRawRotorPosition(rotorPosition);
        m_armEncoderSim.setRawPosition(rotorPosition / ArmConstants.GEARING); // Divide to account for gearing
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
     * @param targetPosition The desired arm position in degrees.
     */
    public void setArmPositionDegrees(double targetPosition) {
        // Ensure position is within bounds
        if (targetPosition < ArmConstants.MIN_ANGLE_DEGREES) {
            targetPosition = ArmConstants.MIN_ANGLE_DEGREES;
        } else if (targetPosition > ArmConstants.MAX_ANGLE_DEGREES) {
            targetPosition = ArmConstants.MAX_ANGLE_DEGREES;
        }

        if ((getElevatorPositionMeters() < ElevatorConstants.SAFETY_POSITION_METERS
                || getElevatorSetpointMeters() < ElevatorConstants.SAFETY_POSITION_METERS)
                && targetPosition > ArmConstants.SAFETY_ANGLE_DOWNWARD_DEGREES) {
            targetPosition = ArmConstants.SAFETY_ANGLE_DOWNWARD_DEGREES;
        } else if ((getElevatorPositionMeters() > 0.0598 || getElevatorSetpointMeters() > 0.0598)
                && targetPosition < ArmConstants.SAFETY_ANGLE_UPWARD_DEGREES) {
            targetPosition = ArmConstants.SAFETY_ANGLE_UPWARD_DEGREES;
        }

        double targetPositionRot = (targetPosition / 360.0); // Convert degrees to rotations

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
            case ZERO -> ArmConstants.ZERO_ANGLE_DEGREES;
            case REST -> ArmConstants.REST_ANGLE_DEGREES;
            case CORAL_L1 -> ArmConstants.L1_ANGLE_DEGREES;
            case CORAL_L2 -> ArmConstants.L2_ANGLE_DEGREES;
            case CORAL_L3 -> ArmConstants.L3_ANGLE_DEGREES;
            case CORAL_L4 -> ArmConstants.L4_ANGLE_DEGREES;
            case PROCESSOR -> ArmConstants.PROCESSOR_ANGLE_DEGREES;
            case ALGAE_HIGH -> ArmConstants.HIGH_ALGAE_ANGLE_DEGREES;
            case ALGAE_LOW -> ArmConstants.LOW_ALGAE_ANGLE_DEGREES;
            case BARGE -> ArmConstants.BARGE_ANGLE_DEGREES;
            default -> ArmConstants.MIN_ANGLE_DEGREES;
        };
        setArmPositionDegrees(degrees);
    }

    /**
     * Gets the current arm position as an enum based on predefined angles.
     *
     * @deprecated This method is inaccurate as multiple states share the same degree value but differ in elevator position,
     * use the getElevatorArmPositionEnum() in the ElevatorSubsystem instead
     * @return The current arm position as an ElevatorPosition enum.
     */
    @Deprecated
    public ElevatorPosition getArmPositionEnum() {
        double positionDeg = getArmPositionDegrees();
        if (Math.abs(positionDeg - ArmConstants.ZERO_ANGLE_DEGREES) < ArmConstants.TARGET_ERROR) {
            return ElevatorPosition.ZERO;
        } else if (Math.abs(positionDeg - ArmConstants.REST_ANGLE_DEGREES) < ArmConstants.TARGET_ERROR) {
            return ElevatorPosition.REST;
        } else if (Math.abs(positionDeg - ArmConstants.L1_ANGLE_DEGREES) < ArmConstants.TARGET_ERROR) {
            return ElevatorPosition.CORAL_L1;
        } else if (Math.abs(positionDeg - ArmConstants.L2_ANGLE_DEGREES) < ArmConstants.TARGET_ERROR) {
            return ElevatorPosition.CORAL_L2;
        } else if (Math.abs(positionDeg - ArmConstants.L3_ANGLE_DEGREES) < ArmConstants.TARGET_ERROR) {
            return ElevatorPosition.CORAL_L3;
        } else if (Math.abs(positionDeg - ArmConstants.L4_ANGLE_DEGREES) < ArmConstants.TARGET_ERROR) {
            return ElevatorPosition.CORAL_L4;
        } else if (Math.abs(positionDeg - ArmConstants.PROCESSOR_ANGLE_DEGREES) < ArmConstants.TARGET_ERROR) {
            return ElevatorPosition.PROCESSOR;
        } else if (Math.abs(positionDeg - ArmConstants.HIGH_ALGAE_ANGLE_DEGREES) < ArmConstants.TARGET_ERROR) {
            return ElevatorPosition.ALGAE_HIGH;
        } else if (Math.abs(positionDeg - ArmConstants.LOW_ALGAE_ANGLE_DEGREES) < ArmConstants.TARGET_ERROR) {
            return ElevatorPosition.ALGAE_LOW;
        } else if (Math.abs(positionDeg - ArmConstants.BARGE_ANGLE_DEGREES) < ArmConstants.TARGET_ERROR) {
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
        return m_armMotor.getClosedLoopReference().getValueAsDouble() * 360;
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
    public double getElevatorPositionMeters() {
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

    public boolean hasReachedAngle() {
        return Math.abs(getArmPositionDegrees() - getArmSetpointDegrees()) < ArmConstants.TARGET_ERROR;
    }
}