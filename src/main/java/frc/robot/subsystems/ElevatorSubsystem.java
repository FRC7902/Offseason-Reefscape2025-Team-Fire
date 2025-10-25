// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.end_effector.IntakeCommand.IntakeMode;
import frc.robot.RobotContainer;

public class ElevatorSubsystem extends SubsystemBase {

    /**
     * Elevator position enum
     */
    public enum ElevatorPosition {
        ZERO,
        REST,
        CORAL_L1,
        CORAL_L2,
        CORAL_L3,
        CORAL_L4,
        PROCESSOR,
        ALGAE_HIGH,
        ALGAE_LOW,
        BARGE,
        UNKNOWN
    }
    /**
     * Leader TalonFX motor
     */
    private final TalonFX m_leaderMotor = new TalonFX(ElevatorConstants.LEADER_MOTOR_CAN_ID);

    /**
     * Follower TalonFX motor
     */
    private final TalonFX m_followerMotor = new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_CAN_ID);

    /**
     * TalonFX motor configuration
     */
    private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();

    /**
     * Motor voltage request object
     */
    private VoltageOut m_voltageRequest = new VoltageOut(0);

    /**
     * Motion magic request object
     */
    private MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0);

    /**
     * Simulated elevator instance object
     */
    private final ElevatorSim m_elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(2),
            ElevatorConstants.GEARING,
            ElevatorConstants.CARRIAGE_MASS,
            ElevatorConstants.DRUM_RADIUS,
            ElevatorConstants.MIN_HEIGHT_METERS,
            ElevatorConstants.MAX_HEIGHT_METERS,
            true,
            ElevatorConstants.HEIGHT_METERS,
            0, 0
    );

    /**
     * Mechanism2d instance for the elevator
     */
    private final Mechanism2d m_mech2d = new Mechanism2d(
            Units.inchesToMeters(50),
            Units.inchesToMeters(100)
    );

    /**
     * Elevator base mechanism root
     */
    private final MechanismRoot2d m_elevatorBase2d = m_mech2d.getRoot(
            "ElevatorRoot",
            Units.inchesToMeters(25),
            Units.inchesToMeters(0.5)
    );

    /**
     * Elevator carriage mechanism root
     */
    private final MechanismRoot2d m_elevatorCarriageRoot2d = m_mech2d.getRoot(
            "ElevatorCarriage",
            Units.inchesToMeters(25),
            Units.inchesToMeters(0.5) + ElevatorConstants.MIN_HEIGHT_METERS
    );

    /**
     * Main elevator ligament, represents the whole elevator
     */
    private final MechanismLigament2d m_elevator2d = m_elevatorBase2d.append(
            new MechanismLigament2d(
                    "Elevator",
                    ElevatorConstants.HEIGHT_METERS,
                    90,
                    7,
                    new Color8Bit(Color.kAntiqueWhite)
            )
    );

    /**
     * Elevator carriage ligament
     */
    private final MechanismLigament2d m_elevatorCarriageLigament2d = m_elevatorCarriageRoot2d.append(
            new MechanismLigament2d(
                    "ElevatorCarriage",
                    ElevatorConstants.CARRIAGE_HEIGHT_METERS,
                    270,
                    15,
                    new Color8Bit(Color.kDarkRed)
            )
    );

    /**
     * Target setpoint for the elevator in meters
     */
    private double m_elevatorSetPointMeters = ElevatorConstants.MIN_HEIGHT_METERS;

    private ElevatorPosition m_ElevatorPositionEnum = ElevatorPosition.REST;

    private IntakeMode m_IntakeMode = null;

    /**
     * SysId routine object to determine S, V, and A constants
     */
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    (volts) -> m_leaderMotor.setControl(m_voltageRequest.withOutput(volts.in(Volts))),
                    null,
                    this
            )
    );

    public ElevatorSubsystem() {
        if (RobotBase.isSimulation()) {
            SmartDashboard.putData("Elevator - Simulation", m_mech2d);
        }

        if (ElevatorConstants.TUNING_MODE_ENABLED) {
            SmartDashboard.putNumber("Elevator - PID_P", ElevatorConstants.PID_P);
            SmartDashboard.putNumber("Elevator - PID_I", ElevatorConstants.PID_I);
            SmartDashboard.putNumber("Elevator - PID_D", ElevatorConstants.PID_D);
            SmartDashboard.putNumber("Elevator - FF_G", ElevatorConstants.FF_G);
        }

        // Set motor configuration
        m_motorConfig.MotorOutput.Inverted = RobotBase.isSimulation() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set slot 0 values
        m_motorConfig.Slot0.kP = ElevatorConstants.PID_P;
        m_motorConfig.Slot0.kI = ElevatorConstants.PID_I;
        m_motorConfig.Slot0.kD = ElevatorConstants.PID_D;
        m_motorConfig.Slot0.kS = ElevatorConstants.FF_S;
        m_motorConfig.Slot0.kV = ElevatorConstants.FF_V;
        m_motorConfig.Slot0.kA = ElevatorConstants.FF_A;
        m_motorConfig.Slot0.kG = ElevatorConstants.FF_G;

        // Set gravity type
        m_motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // Set motion magic
        m_motorConfig.MotionMagic.MotionMagicCruiseVelocity =
                ElevatorConstants.MAX_VELOCITY;
        m_motorConfig.MotionMagic.MotionMagicAcceleration = 160;

        // Set safety limits
        /*m _motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                ElevatorConstants.kElevatorMaxHeightMeters / ElevatorConstants.kElevatorMetersPerMotorRotation;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
 */
        // Set current limits
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;

        // Set current limits
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;

        // Set follower
        m_followerMotor.setControl(new Follower(m_leaderMotor.getDeviceID(), true));

        // Apply configuration to motors
        m_leaderMotor.getConfigurator().apply(m_motorConfig);
        m_followerMotor.getConfigurator().apply(m_motorConfig);

        // Set update frequencies
        m_leaderMotor.getPosition().setUpdateFrequency(50);
        m_leaderMotor.getVelocity().setUpdateFrequency(50);
        m_leaderMotor.getDutyCycle().setUpdateFrequency(50);
        m_leaderMotor.getMotorVoltage().setUpdateFrequency(50);
        m_leaderMotor.getTorqueCurrent().setUpdateFrequency(50);
    }

    /**
     * Sets the elevator position based on the provided ElevatorPosition enum.
     *
     * @param positionEnum The desired elevator position enum.
     */
    public void setElevatorPositionEnum(ElevatorPosition positionEnum) {
        m_elevatorSetPointMeters = switch (positionEnum) {
            case ZERO -> ElevatorConstants.MIN_HEIGHT_METERS;
            case REST -> ElevatorConstants.REST_HEIGHT_METERS;
            case CORAL_L1 -> ElevatorConstants.L1_HEIGHT_METERS;
            case CORAL_L2 -> ElevatorConstants.L2_HEIGHT_METERS;
            case CORAL_L3 -> ElevatorConstants.L3_HEIGHT_METERS;
            case CORAL_L4 -> ElevatorConstants.L4_HEIGHT_METERS;
            case PROCESSOR -> ElevatorConstants.PROCESSOR_HEIGHT_METERS;
            case ALGAE_HIGH -> ElevatorConstants.HIGH_ALGAE_HEIGHT_METERS;
            case ALGAE_LOW -> ElevatorConstants.LOW_ALGAE_HEIGHT_METERS;
            case BARGE -> ElevatorConstants.BARGE_HEIGHT_METERS;
            default -> ElevatorConstants.MIN_HEIGHT_METERS;
        };

        setElevatorPositionMeters(m_elevatorSetPointMeters);
    }

    public void setElevatorPositionEnumOperator(ElevatorPosition positionEnum) {
        m_ElevatorPositionEnum = positionEnum;
    }
    public void setIntakeMode(){
        switch (m_ElevatorPositionEnum){
            case CORAL_L1:
                m_IntakeMode = IntakeMode.CORAL;
            case CORAL_L2:
                m_IntakeMode = IntakeMode.CORAL;
            case CORAL_L3:
                m_IntakeMode = IntakeMode.CORAL;
            case CORAL_L4:
                m_IntakeMode = IntakeMode.CORAL;
            case ALGAE_HIGH:
                m_IntakeMode = IntakeMode.ALGAE;
            case ALGAE_LOW:
                m_IntakeMode = IntakeMode.ALGAE;
            default:
                m_IntakeMode = null;
        }
    }
    public IntakeMode getIntakeMode() {
        return m_IntakeMode;
    }
    public ElevatorPosition getElevatorPositionEnumOperator() {
        return m_ElevatorPositionEnum;
    }
    /**
     * Sets the elevator position in meters.
     *
     * @param positionMeters The desired elevator position in meters.
     */
    public void setElevatorPositionMeters(double positionMeters) {
        double currentPosMeters = getElevatorPositionMeters();
        if (positionMeters < ElevatorConstants.MIN_HEIGHT_METERS) {
            positionMeters = ElevatorConstants.MIN_HEIGHT_METERS;
        } else if (positionMeters > ElevatorConstants.MAX_HEIGHT_METERS) {
            positionMeters = ElevatorConstants.MAX_HEIGHT_METERS;
        }

        // if (getArmPositionDegrees() < ArmConstants.SAFETY_ANGLE_UPWARD_DEGREES) {
        //     positionMeters = currentPosMeters;
        // }

        // upper line causing autonomous elevator failure

        m_elevatorSetPointMeters = positionMeters;

        double positionRotations = (m_elevatorSetPointMeters - (RobotBase.isSimulation() ? 0 : ElevatorConstants.MIN_HEIGHT_METERS)) / ElevatorConstants.METERS_PER_MOTOR_ROTATION;
        m_motionMagicRequest = m_motionMagicRequest.withPosition(positionRotations).withSlot(0);
        m_leaderMotor.setControl(m_motionMagicRequest);
    }

    /**
     * Holds the elevator in place by applying a zero velocity control.
     */
    public void holdElevatorInPlace() {
        m_leaderMotor.setControl(m_voltageRequest.withOutput(0));
    }

    /**
     * Gets the current elevator position in meters.
     *
     * @return The current elevator position in meters.
     */
    public double getElevatorPositionMeters() {
        double positionRotations = m_leaderMotor.getPosition().getValueAsDouble();
        return positionRotations * ElevatorConstants.METERS_PER_MOTOR_ROTATION + (RobotBase.isSimulation() ? 0 : ElevatorConstants.MIN_HEIGHT_METERS);
    }

    /**
     * Gets the current elevation position as a scale from 0 to 1, where 0 is the minimum height and 1 is the maximum height.
     *
     * @return The current elevator position as a scale from 0 to 1.
     */
    public double getElevatorPositionScale() {
        return (getElevatorPositionMeters() - ElevatorConstants.MIN_HEIGHT_METERS) / (ElevatorConstants.MAX_HEIGHT_METERS - ElevatorConstants.MIN_HEIGHT_METERS);
    }

    /**
     * Gets the current elevator position in the ElevatorPosition enum. Returns UKNOWN if not in a valid position.
     *
     * @return The current elevator position as an ElevatorPosition enum.
     */
    public ElevatorPosition getElevatorPositionEnum() {
        double currentHeightMeters = getElevatorPositionMeters();

        if (Math.abs(currentHeightMeters - ElevatorConstants.MIN_HEIGHT_METERS) < ElevatorConstants.TARGET_ERROR * 2) {
            return ElevatorPosition.ZERO;
        } else if (Math.abs(currentHeightMeters - ElevatorConstants.REST_HEIGHT_METERS) < ElevatorConstants.TARGET_ERROR * 2) {
            return ElevatorPosition.REST;
        } else if (Math.abs(currentHeightMeters - ElevatorConstants.L1_HEIGHT_METERS) < ElevatorConstants.TARGET_ERROR * 2) {
            return ElevatorPosition.CORAL_L1;
        } else if (Math.abs(currentHeightMeters - ElevatorConstants.L2_HEIGHT_METERS) < ElevatorConstants.TARGET_ERROR * 2) {
            return ElevatorPosition.CORAL_L2;
        } else if (Math.abs(currentHeightMeters - ElevatorConstants.L3_HEIGHT_METERS) < ElevatorConstants.TARGET_ERROR * 2) {
            return ElevatorPosition.CORAL_L3;
        } else if (Math.abs(currentHeightMeters - ElevatorConstants.L4_HEIGHT_METERS) < ElevatorConstants.TARGET_ERROR * 2) {
            return ElevatorPosition.CORAL_L4;
        } else if (Math.abs(currentHeightMeters - ElevatorConstants.PROCESSOR_HEIGHT_METERS) < ElevatorConstants.TARGET_ERROR * 2) {
            return ElevatorPosition.PROCESSOR;
        } else if (Math.abs(currentHeightMeters - ElevatorConstants.HIGH_ALGAE_HEIGHT_METERS) < ElevatorConstants.TARGET_ERROR * 2) {
            return ElevatorPosition.ALGAE_HIGH;
        } else if (Math.abs(currentHeightMeters - ElevatorConstants.LOW_ALGAE_HEIGHT_METERS) < ElevatorConstants.TARGET_ERROR * 2) {
            return ElevatorPosition.ALGAE_LOW;
        } else if (Math.abs(currentHeightMeters - ElevatorConstants.BARGE_HEIGHT_METERS) < ElevatorConstants.TARGET_ERROR * 2) {
            return ElevatorPosition.BARGE;
        } else {
            return ElevatorPosition.UNKNOWN;
        }
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

    /**
     * Update telemetry, including the mechanism visualization
     */
    public void updateTelemetry() {
        if (getElevatorPositionMeters() > ElevatorConstants.HEIGHT_METERS) {
            m_elevator2d.setLength(getElevatorPositionMeters());
        } else {
            m_elevator2d.setLength(ElevatorConstants.HEIGHT_METERS);
        }
        m_elevatorCarriageRoot2d.setPosition(Units.inchesToMeters(25), Units.inchesToMeters(0.5) + getElevatorPositionMeters());
    }

    /**
     * Gets the current setpoint for the elevator in meters.
     *
     * @return The current elevator setpoint in meters.
     */
    public double getSetpoint() {
        return m_elevatorSetPointMeters;
    }

    /**
     * Gets the Mechanism2d instance for the elevator subsystem.
     *
     * @return The Mechanism2d instance.
     */
    public Mechanism2d getMechanism2d() {
        return m_mech2d;
    }

    /**
     * Gets the position of the arm in degrees
     *
     * @return The position of the arm in degrees
     */
    private double getArmPositionDegrees() {
        return RobotContainer.m_armSubsystem.getArmPositionDegrees();
    }

    public boolean hasReachedSetpoint() {
        return Math.abs(getElevatorPositionMeters() - getSetpoint()) < ElevatorConstants.TARGET_ERROR;
    }

    @Override
    public void periodic() {
        if (m_leaderMotor.getClosedLoopReference().getValueAsDouble() * ElevatorConstants.METERS_PER_MOTOR_ROTATION > ElevatorConstants.MIN_HEIGHT_METERS &&
                m_leaderMotor.getClosedLoopReference().getValueAsDouble() * ElevatorConstants.METERS_PER_MOTOR_ROTATION < ElevatorConstants.ZERO_THRESHOLD &&
                m_leaderMotor.getPosition().getValueAsDouble() * ElevatorConstants.METERS_PER_MOTOR_ROTATION > ElevatorConstants.MIN_HEIGHT_METERS &&
                m_leaderMotor.getPosition().getValueAsDouble() * ElevatorConstants.METERS_PER_MOTOR_ROTATION < ElevatorConstants.ZERO_THRESHOLD)
            m_leaderMotor.setVoltage(0);
        else {
            m_leaderMotor.setControl(m_motionMagicRequest);
        }

        if (ElevatorConstants.TUNING_MODE_ENABLED) {
            ElevatorConstants.PID_P = SmartDashboard.getNumber("Elevator - PID_P", ElevatorConstants.PID_P);
            ElevatorConstants.PID_I = SmartDashboard.getNumber("Elevator - PID_I", ElevatorConstants.PID_I);
            ElevatorConstants.PID_D = SmartDashboard.getNumber("Elevator - PID_D", ElevatorConstants.PID_D);
            ElevatorConstants.FF_G = SmartDashboard.getNumber("Elevator - FF_G", ElevatorConstants.FF_G);

            m_motorConfig.Slot0.kP = ElevatorConstants.PID_P;
            m_motorConfig.Slot0.kI = ElevatorConstants.PID_I;
            m_motorConfig.Slot0.kD = ElevatorConstants.PID_D;
            m_motorConfig.Slot0.kG = ElevatorConstants.FF_G;

            m_leaderMotor.getConfigurator().apply(m_motorConfig);
        }

        SmartDashboard.putNumber("Elevator - Position (m)", getElevatorPositionMeters());
        SmartDashboard.putNumber("Elevator - Setpoint (m)", m_elevatorSetPointMeters);
        SmartDashboard.putString("Elevator - Position Enum", getElevatorPositionEnum().toString());
        SmartDashboard.putNumber("Elevator - Stator Current", m_leaderMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator - Leader Motor Pos", m_leaderMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator - Follower Motor Pos", m_followerMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator - Closed loop error (m)", m_leaderMotor.getClosedLoopError().getValueAsDouble() * ElevatorConstants.METERS_PER_MOTOR_ROTATION);

        SmartDashboard.putBoolean("Elevator - At Setpoint", hasReachedSetpoint());

        updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_elevatorSim.setInputVoltage(m_leaderMotor.getMotorVoltage().getValueAsDouble());
        m_elevatorSim.update(0.02);

        final double positionRotations = m_elevatorSim.getPositionMeters() / ElevatorConstants.METERS_PER_MOTOR_ROTATION;
        final double velocityRPS = m_elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.METERS_PER_MOTOR_ROTATION;

        m_leaderMotor.getSimState().setRawRotorPosition(positionRotations);
        m_leaderMotor.getSimState().setRotorVelocity(velocityRPS);

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps())
        );
    }
}


