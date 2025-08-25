package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FunnelIndexerSubsystem extends SubsystemBase {
    private final SparkMax m_indexerLeftMotor;
    private final SparkMax m_indexerRightMotor;
    private final SparkMax m_indexerkickerMotor;
    private final DigitalInput m_shallowBeamBreak;
    private final DigitalInput m_deepBeamBreak;

   // ...existing code...
    public FunnelIndexerSubsystem() {
        m_indexerLeftMotor = new SparkMax(Constants.FunnelIndexerConstants.kLeftMotorCANID, MotorType.kBrushless);   // NEO 550
        m_indexerRightMotor = new SparkMax(Constants.FunnelIndexerConstants.kRightMotorCANID, MotorType.kBrushless);  // NEO 550
        m_indexerkickerMotor = new SparkMax(Constants.FunnelIndexerConstants.kKickerMotorCANID, MotorType.kBrushless); // NEO 550

        m_shallowBeamBreak = new DigitalInput(0);
        m_deepBeamBreak = new DigitalInput(1);

        // Kicker motor spins all the time
        m_indexerkickerMotor.set(Constants.FunnelIndexerConstants.m_fullSpeed);
    }
// ...existing code...

    // Optionally, control kicker motor if needed
    public void setKickerPower(double speed) {
        m_indexerkickerMotor.set(speed);
    }

    public boolean isShallowBeamBroken() {
        return !m_shallowBeamBreak.get();
    }

    public boolean isDeepBeamBroken() {
        return !m_deepBeamBreak.get();
    }

    public boolean CanShoot() {
        return isShallowBeamBroken() && isDeepBeamBroken();
    }    
    public void setPower(double speed) {
        m_indexerLeftMotor.set(speed);
        m_indexerRightMotor.set(-speed); // Invert right motor to ensure both motors spin in the same direction
    }

    @Override
    public void periodic() {
        // Add periodic code here if needed
    }
}