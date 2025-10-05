// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {

    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;
    private final MotorOutputConfigs m_motorOutputConfig;
    private boolean m_hasAlgae;
    private boolean m_hasCoral;

    private final DigitalInput m_coralBeamBreak;
    // private final DigitalInput m_algaeProximitySensor;

    private final Debouncer m_algaeDebouncer;

    /**
     * Creates a new AlgaeCoralIndexerSubsystem.
     */
    public EndEffectorSubsystem() {
        m_motor = new TalonFX(EndEffectorConstants.MOTOR_CAN_ID);

        m_motorConfig = new TalonFXConfiguration();
        m_motorOutputConfig = new MotorOutputConfigs();

        m_coralBeamBreak = new DigitalInput(EndEffectorConstants.CORAL_BEAM_BREAK_PORT_ID);
        // m_algaeProximitySensor = new DigitalInput(EndEffectorConstants.ALGAE_PROXIMITY_SENSOR_PORT_ID);

        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = EndEffectorConstants.MOTOR_STATOR_CURRENT_LIMIT;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = EndEffectorConstants.MOTOR_SUPPLY_CURRENT_LIMIT;

        m_motorOutputConfig.withNeutralMode(NeutralModeValue.Brake);
        m_motorConfig.withMotorOutput(m_motorOutputConfig);

        m_motor.getConfigurator().apply(m_motorConfig);

        m_algaeDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kRising);
        m_hasAlgae = false;
        m_hasCoral = false;
    }

    public boolean hasAlgae() {
        return m_hasAlgae;
    }

    public void setHasCoral(boolean hasCoral) {
        m_hasCoral = hasCoral;
    }

    public boolean hasCoral() {
        return m_hasCoral;
    }

    public boolean isCoralBeamBreakBroken() {
        return !m_coralBeamBreak.get();
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    public double getSupplyCurrent() {
        return m_motor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("End Effector — Has Coral", hasCoral());
        SmartDashboard.putBoolean("End Effector - Has Algae", hasAlgae());
        SmartDashboard.putNumber("Supply Current EndEffector", getSupplyCurrent());
        SmartDashboard.putNumber("EndEffector Motor Velocity", m_motor.getVelocity().getValueAsDouble());

        // Debouncer to detect consistent current spike (between low and high) for longer than time (t)
        m_hasAlgae = m_algaeDebouncer.calculate(
                EndEffectorConstants.ALGAE_INTAKE_STALL_DETECTION_CURRENT_LOW < getSupplyCurrent()
                        && getSupplyCurrent() < EndEffectorConstants.ALGAE_INTAKE_STALL_DETECTION_CURRENT_HIGH);
    }
}
