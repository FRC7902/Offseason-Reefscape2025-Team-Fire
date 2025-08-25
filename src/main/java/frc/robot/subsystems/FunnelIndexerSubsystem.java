package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FunnelIndexerConstants;

public class FunnelIndexerSubsystem extends SubsystemBase {
    private final SparkMax m_indexerLeftMotor;
    private final SparkMax m_indexerRightMotor;
    private final SparkMax m_indexerkickerMotor;
    private final DigitalInput m_shallowBeamBreak;
    private final DigitalInput m_deepBeamBreak;

    public FunnelIndexerSubsystem() {
        m_indexerLeftMotor = new SparkMax(FunnelIndexerConstants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless); // NEO 550
        m_indexerRightMotor = new SparkMax(FunnelIndexerConstants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless); // NEO 550
        m_indexerkickerMotor = new SparkMax(FunnelIndexerConstants.KICKER_MOTOR_CAN_ID, MotorType.kBrushless); // NEO 550

        m_shallowBeamBreak = new DigitalInput(0);
        m_deepBeamBreak = new DigitalInput(1);

        // Kicker motor spins all the time
        m_indexerkickerMotor.set(FunnelIndexerConstants.FULL_SPEED);
    }

    public void setKickerSpeed(double speed) {
        m_indexerkickerMotor.set(speed);
    }

    public boolean isShallowBeamBroken() {
        return !m_shallowBeamBreak.get();
    }

    public boolean isDeepBeamBroken() {
        return !m_deepBeamBreak.get();
    }

    public void setIndexerSpeed(double speed) {
        m_indexerLeftMotor.set(speed);
        m_indexerRightMotor.set(-speed); // Invert right motor to ensure both motors spin in the same direction
    }

    public void stop() {
        setIndexerSpeed(0);
        m_indexerLeftMotor.stopMotor();
        m_indexerRightMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // Add periodic code here if needed
    }
}