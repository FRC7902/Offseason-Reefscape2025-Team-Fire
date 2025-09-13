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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer;

public class ElevatorSubsystem extends SubsystemBase {
    
    /** Elevator position enum */
    public enum ElevatorPosition {
        ZERO,
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

    /** Leader TalonFX motor */
    private final TalonFX m_leaderMotor = new TalonFX(ElevatorConstants.kElevatorLeaderCANID);

    /** Follower TalonFX motor */
    private final TalonFX m_followerMotor = new TalonFX(ElevatorConstants.kElevatorFollowerCANID);

    /** TalonFX motor configuration */
    private final TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();

    /** Motor voltage request object */
    private VoltageOut m_voltageRequest = new VoltageOut(0);

    /** Motion magic request object */
    private MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0);

    /** Simulated elevator instance object */
    private final ElevatorSim m_elevatorSim = new ElevatorSim(
        DCMotor.getKrakenX60(2),
        ElevatorConstants.kElevatorGearing,
        ElevatorConstants.kElevatorCarriageMass,
        ElevatorConstants.kElevatorDrumRadius,
        ElevatorConstants.kElevatorMinHeightMeters,
        ElevatorConstants.kElevatorMaxHeightMeters,
        true,
        ElevatorConstants.kElevatorHeightMeters,
        0, 0
    );

    /** Mechanism2d instance for the elevator */
    private final Mechanism2d m_mech2d = new Mechanism2d(
        Units.inchesToMeters(50), 
        Units.inchesToMeters(100)
    );

    /** Elevator base mechanism root */
    private final MechanismRoot2d m_elevatorBase2d = m_mech2d.getRoot(
        "ElevatorRoot", 
        Units.inchesToMeters(25), 
        Units.inchesToMeters(0.5)
    );

    /** Elevator carriage mechanism root */
    private final MechanismRoot2d m_elevatorCarriageRoot2d = m_mech2d.getRoot(
        "ElevatorCarriage",
        Units.inchesToMeters(25),
        Units.inchesToMeters(0.5) + ElevatorConstants.kElevatorMinHeightMeters
    );

    /** Main elevator ligament, represents the whole elevator */
    private final MechanismLigament2d m_elevator2d = m_elevatorBase2d.append(
        new MechanismLigament2d(
            "Elevator",
            ElevatorConstants.kElevatorHeightMeters,
            90,
            7,
            new Color8Bit(Color.kAntiqueWhite)
        ) 
    );

    /** Elevator carriage ligament */
    private final MechanismLigament2d m_elevatorCarriageLigament2d = m_elevatorCarriageRoot2d.append(
        new MechanismLigament2d(
            "ElevatorCarriage",
            ElevatorConstants.kElevatorCarriageHeightMeters,
            270,
            15,
            new Color8Bit(Color.kDarkRed)
        )
    );

    /** Target setpoint for the elevator in meters */
    private double m_elevatorSetPointMeters = ElevatorConstants.kElevatorMinHeightMeters;

    /** SysId routine object to determine S, V, and A constants */
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
            SmartDashboard.putData("ElevatorSimulation", m_mech2d);
        }

        if (ElevatorConstants.kTuningMode) {
            SmartDashboard.putNumber("Elevator P", ElevatorConstants.kElevatorP);
            SmartDashboard.putNumber("Elevator I", ElevatorConstants.kElevatorI);
            SmartDashboard.putNumber("Elevator D", ElevatorConstants.kElevatorD);
            SmartDashboard.putNumber("Elevator G", ElevatorConstants.kElevatorG);
        }

        // Set motor configuration
        m_motorConfig.MotorOutput.Inverted = RobotBase.isSimulation() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set slot 0 values
        m_motorConfig.Slot0.kP = ElevatorConstants.kElevatorP;
        m_motorConfig.Slot0.kI = ElevatorConstants.kElevatorI;
        m_motorConfig.Slot0.kD = ElevatorConstants.kElevatorD;
        m_motorConfig.Slot0.kS = ElevatorConstants.kElevatorS;
        m_motorConfig.Slot0.kV = ElevatorConstants.kElevatorV;
        m_motorConfig.Slot0.kA = ElevatorConstants.kElevatorA;
        m_motorConfig.Slot0.kG = ElevatorConstants.kElevatorG;

        // Set gravity type
        m_motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // Set motion magic
        m_motorConfig.MotionMagic.MotionMagicCruiseVelocity =
                ElevatorConstants.kElevatorMaxVelocity;
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
        m_motorConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kElevatorStatorCurrentLimit;

        // Set current limits
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.kElevatorSupplyCurrentLimit; 

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
     * @param position The desired elevator position enum.
     */
    public void setElevatorPositionEnum(ElevatorPosition positionEnum) {
        m_elevatorSetPointMeters = switch (positionEnum) {
            case ZERO -> ElevatorConstants.kElevatorMinHeightMeters;
            case CORAL_L1 -> ElevatorConstants.kElevatorCoralLevel1Height;
            case CORAL_L2 -> ElevatorConstants.kElevatorCoralLevel2Height;
            case CORAL_L3 -> ElevatorConstants.kElevatorCoralLevel3Height;
            case CORAL_L4 -> ElevatorConstants.kElevatorCoralLevel4Height;
            case PROCESSOR -> ElevatorConstants.kElevatorProcessorHeight;
            case ALGAE_HIGH -> ElevatorConstants.kElevatorAlgaeHighHeight;
            case ALGAE_LOW -> ElevatorConstants.kElevatorAlgaeLowHeight;
            case BARGE -> ElevatorConstants.kElevatorBargeHeight;
            default -> ElevatorConstants.kElevatorMinHeightMeters;
        };

        setElevatorPositionMeters(m_elevatorSetPointMeters);
    }

    /**
     * Sets the elevator position in meters.
     * 
     * @param positionMeters The desired elevator position in meters.
     */
    public void setElevatorPositionMeters(double positionMeters) {
        double currentPosMeters = getElevatorPositionMeters();
        if(positionMeters < ElevatorConstants.kElevatorMinHeightMeters) {
            positionMeters = ElevatorConstants.kElevatorMinHeightMeters;
        } else if(positionMeters > ElevatorConstants.kElevatorMaxHeightMeters) {
            positionMeters = ElevatorConstants.kElevatorMaxHeightMeters;
        }
        if (getArmPositionDegrees() < ElevatorConstants.kAngleBad) {
            positionMeters = currentPosMeters;
        }

        m_elevatorSetPointMeters = positionMeters;

        double positionRotations = (m_elevatorSetPointMeters - (RobotBase.isSimulation() ? 0 : ElevatorConstants.kElevatorMinHeightMeters)) / ElevatorConstants.kElevatorMetersPerMotorRotation;
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
        return positionRotations * ElevatorConstants.kElevatorMetersPerMotorRotation + (RobotBase.isSimulation() ? 0 : ElevatorConstants.kElevatorMinHeightMeters);
    }

    /**
     * Gets the current elevator position in the ElevatorPosition enum. Returns UKNOWN if not in a valid position.
     * 
     * @return The current elevator position as an ElevatorPosition enum.
     */
    public ElevatorPosition getElevatorPositionEnum() {
        double positionMeters = getElevatorPositionMeters();
        if (positionMeters < ElevatorConstants.kElevatorMinHeightMeters + 0.01) {
            return ElevatorPosition.ZERO;
        } else if (positionMeters < ElevatorConstants.kElevatorCoralLevel1Height + ElevatorConstants.kElevatorTargetError*2) {
            return ElevatorPosition.CORAL_L1;
        } else if (positionMeters < ElevatorConstants.kElevatorCoralLevel2Height + ElevatorConstants.kElevatorTargetError*2) {
            return ElevatorPosition.CORAL_L2;
        } else if (positionMeters < ElevatorConstants.kElevatorCoralLevel3Height + ElevatorConstants.kElevatorTargetError*2) {
            return ElevatorPosition.CORAL_L3;
        } else if (positionMeters < ElevatorConstants.kElevatorCoralLevel4Height + ElevatorConstants.kElevatorTargetError*2) {
            return ElevatorPosition.CORAL_L4;
        } else if (positionMeters < ElevatorConstants.kElevatorProcessorHeight + ElevatorConstants.kElevatorTargetError*2) {
            return ElevatorPosition.PROCESSOR;
        } else if (positionMeters < ElevatorConstants.kElevatorAlgaeHighHeight + ElevatorConstants.kElevatorTargetError*2) {
            return ElevatorPosition.ALGAE_HIGH;
        } else if (positionMeters < ElevatorConstants.kElevatorMaxHeightMeters + ElevatorConstants.kElevatorTargetError*2) {
            return ElevatorPosition.ALGAE_LOW;
        } else if (positionMeters < ElevatorConstants.kElevatorBargeHeight + ElevatorConstants.kElevatorTargetError*2) {
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
        if (getElevatorPositionMeters() > ElevatorConstants.kElevatorHeightMeters) {
            m_elevator2d.setLength(getElevatorPositionMeters());
        } else {
            m_elevator2d.setLength(ElevatorConstants.kElevatorHeightMeters);
        }
        m_elevatorCarriageRoot2d.setPosition(Units.inchesToMeters(25), Units.inchesToMeters(0.5)+getElevatorPositionMeters());
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

    @Override
    public void periodic() {
        if (m_leaderMotor.getClosedLoopReference().getValueAsDouble()*ElevatorConstants.kElevatorMetersPerMotorRotation > ElevatorConstants.kElevatorMinHeightMeters &&
            m_leaderMotor.getClosedLoopReference().getValueAsDouble()*ElevatorConstants.kElevatorMetersPerMotorRotation < ElevatorConstants.kElevatorZeroThreshold &&
            m_leaderMotor.getPosition().getValueAsDouble()*ElevatorConstants.kElevatorMetersPerMotorRotation > ElevatorConstants.kElevatorMinHeightMeters &&
            m_leaderMotor.getPosition().getValueAsDouble()*ElevatorConstants.kElevatorMetersPerMotorRotation < ElevatorConstants.kElevatorZeroThreshold)
            m_leaderMotor.setVoltage(0);
        else {
            m_leaderMotor.setControl(m_motionMagicRequest);
        }

        if (ElevatorConstants.kTuningMode) {
            ElevatorConstants.kElevatorP = SmartDashboard.getNumber("Elevator P", ElevatorConstants.kElevatorP);
            ElevatorConstants.kElevatorI = SmartDashboard.getNumber("Elevator I", ElevatorConstants.kElevatorI);
            ElevatorConstants.kElevatorD = SmartDashboard.getNumber("Elevator D", ElevatorConstants.kElevatorD);
            ElevatorConstants.kElevatorG = SmartDashboard.getNumber("Elevator G", ElevatorConstants.kElevatorG);

            m_motorConfig.Slot0.kP = ElevatorConstants.kElevatorP;
            m_motorConfig.Slot0.kI = ElevatorConstants.kElevatorI;
            m_motorConfig.Slot0.kD = ElevatorConstants.kElevatorD;
            m_motorConfig.Slot0.kG = ElevatorConstants.kElevatorG;

            m_leaderMotor.getConfigurator().apply(m_motorConfig);
        }
        
        SmartDashboard.putNumber("Elevator Position (m)", getElevatorPositionMeters());
        SmartDashboard.putNumber("Elevator Setpoint (m)", m_elevatorSetPointMeters);
        SmartDashboard.putString("Elevator Position Enum", getElevatorPositionEnum().toString());
        SmartDashboard.putNumber("Elevator Stator Current", m_leaderMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Leader Motor Pos", m_leaderMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Follower Motor Pos", m_followerMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Closed loop error metres", m_leaderMotor.getClosedLoopError().getValueAsDouble() * ElevatorConstants.kElevatorMetersPerMotorRotation);

        updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_elevatorSim.setInputVoltage(m_leaderMotor.getMotorVoltage().getValueAsDouble());
        m_elevatorSim.update(0.02);

        final double positionRotations = m_elevatorSim.getPositionMeters() / ElevatorConstants.kElevatorMetersPerMotorRotation;
        final double velocityRPS = m_elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.kElevatorMetersPerMotorRotation;

        m_leaderMotor.getSimState().setRawRotorPosition(positionRotations);
        m_leaderMotor.getSimState().setRotorVelocity(velocityRPS);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps())
        );
    }
}


