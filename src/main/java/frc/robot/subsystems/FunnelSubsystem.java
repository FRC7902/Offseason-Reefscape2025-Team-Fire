package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FunnelIndexerConstants;

public class FunnelSubsystem extends SubsystemBase {
    private final SparkMax m_indexerLeftMotor;
    private final SparkMax m_indexerRightMotor;
//    private final SparkMax m_kickerMotor;

    private final DigitalInput m_shallowBeamBreak;
    private final DigitalInput m_deepBeamBreak;

    private Boolean m_hasCoral;

    private final Timer m_timer;
    private double m_lastTimeHasCoral;

    public FunnelSubsystem() {
        m_indexerLeftMotor = new SparkMax(FunnelIndexerConstants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless); // NEO 550
        m_indexerRightMotor = new SparkMax(FunnelIndexerConstants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless); // NEO 550
//        m_kickerMotor = new SparkMax(FunnelIndexerConstants.KICKER_MOTOR_CAN_ID, MotorType.kBrushless); // NEO 550

        SparkMaxConfig m_indexerLeftConfig = new SparkMaxConfig();
        SparkMaxConfig m_indexerRightConfig = new SparkMaxConfig();
//        SparkMaxConfig m_kickerConfig = new SparkMaxConfig();

        // Set current limits
        m_indexerLeftConfig.smartCurrentLimit(FunnelIndexerConstants.MOTOR_CURRENT_LIMIT);
        m_indexerRightConfig.smartCurrentLimit(FunnelIndexerConstants.MOTOR_CURRENT_LIMIT);
//        m_kickerConfig.smartCurrentLimit(FunnelIndexerConstants.MOTOR_CURRENT_LIMIT);

        m_indexerLeftConfig.idleMode(IdleMode.kBrake);
        m_indexerRightConfig.idleMode(IdleMode.kBrake);
//        m_kickerConfig.idleMode(IdleMode.kBrake);

        m_indexerLeftMotor.configure(m_indexerLeftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        m_indexerRightMotor.configure(m_indexerRightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
//        m_kickerMotor.configure(m_kickerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        m_shallowBeamBreak = new DigitalInput(FunnelIndexerConstants.SHALLOW_BEAM_BREAK_DIO);
        m_deepBeamBreak = new DigitalInput(FunnelIndexerConstants.DEEP_BEAM_BREAK_DIO);

        m_hasCoral = false;

        m_timer = new Timer();
        m_timer.start();

        m_lastTimeHasCoral = 0;

//        Kicker motor spins all the time
//        m_kickerMotor.set(FunnelIndexerConstants.FULL_SPEED);
    }

//    public void setKickerSpeed(double speed) {
//        m_kickerMotor.set(speed);
//    }

    /**
     * Get the state of the shallow beam break.
     *
     * @return True if the shallow beam break is broken (i.e., something is blocking the beam).
     */
    public boolean isShallowBeamBreakBroken() {
        return !m_shallowBeamBreak.get();
    }

    /**
     * Get the state of the deep beam break.
     *
     * @return True if the deep beam break is broken (i.e., something is blocking the beam).
     */
    public boolean isDeepBeamBreakBroken() {
        return !m_deepBeamBreak.get();
    }

    /**
     * Set the speed of the indexer motors.
     *
     * @param speed The speed to set the indexer motors to, between -1.0 and 1.0.
     */
    public void setIndexerSpeed(double speed) {
        m_indexerLeftMotor.set(speed);
        m_indexerRightMotor.set(-speed); // Invert right motor to ensure both motors spin in the same direction
    }

    /**
     * Stop the indexer motors.
     */
    public void stop() {
        setIndexerSpeed(0);
        m_indexerLeftMotor.stopMotor();
        m_indexerRightMotor.stopMotor();
    }

    /**
     * Check if the funnel currently has coral.
     *
     * @return True if the funnel has coral, false otherwise.
     */
    public Boolean getHasCoral() {
        return m_hasCoral;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Funnel - Shallow Beam Break", isShallowBeamBreakBroken());
        SmartDashboard.putBoolean("Funnel - Deep Beam Break", isDeepBeamBreakBroken());

        SmartDashboard.putBoolean("Funnel - Has Coral", m_hasCoral);

        if (m_hasCoral) {
            if (!isShallowBeamBreakBroken()) {
                double currTime = m_timer.get();
                if (currTime - m_lastTimeHasCoral > 2) {
                    m_hasCoral = false;
                }
            }
        } else {
            if (isShallowBeamBreakBroken() && isDeepBeamBreakBroken()) {
                m_hasCoral = true;
                m_lastTimeHasCoral = m_timer.get();
            }
        }
    }
}