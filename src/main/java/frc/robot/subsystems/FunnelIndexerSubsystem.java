// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelIndexerSubsystem extends SubsystemBase {
    // Member variables
    private DigitalInput m_shallowBeamBreak; // Beam break closer to funnel
    private DigitalInput m_deepBeamBreak; // Beam break closer to exit
    //private TalonFX m_indexMotor = new TalonFX(1); // 1 = CAN ID
    //public TalonFX m_indexMotor;
    public TalonFX m_indexMotor;

 

    /** Creates a new FunnelIndexerSubsystem. */
    public FunnelIndexerSubsystem() {
        // Initialize the beam breaks with their respective DIO channels
        // IMPORTANT: Replace the numbers with your actual DIO channel numbers
        //m_shallowBeamBreak = new DigitalInput(0); 
        //m_deepBeamBreak = new DigitalInput(1); 
    }
    public void setPower(double motorPower) {
      m_indexMotor.set(motorPower);
    }

    public void setSpeed(double speed) {
        m_indexMotor.set(speed);
    }

   

    public boolean isShallowBeamBroken() {
        // DigitalInput.get() returns true when the circuit is open,
        // so we negate it to check for a "broken" beam.
        return !m_shallowBeamBreak.get(); 
    }

    public boolean isDeepBeamBroken() {
        return !m_deepBeamBreak.get();
    }

    /*public boolean hasCoral() {
        // Assumes that if the deep beam break is broken, there is coral ready to be indexed
        return isDeepBeamBroken();
    }*/

    @Override
    public void periodic() {
        //SmartDashboard.putBoolean("Shallow Beam Sensor Broken", isShallowBeamBroken());
        //SmartDashboard.putBoolean("Deep Beam Sensor Broken", isDeepBeamBroken());

        // This logic matches the truth table image more accurately.
        // It's a common strategy for indexing systems.
        //if (deepBroken) {
            // Stop the motor if the coral is at the exit
            //setSpeed(m_stopSpeed);
        /* } else if (shallowBroken) {
            // Slow down the motor once the coral is detected at the entrance
            setSpeed(m_halfSpeed);
        } else if(deepBroken && shallowBroken) {
            // If both sensors are broken, reverse the motor
            setSpeed(m_reverseSpeed);
        }else {
            // Run at full speed if no coral is detected
            setSpeed(m_fullSpeed);
        }*/
    }
}