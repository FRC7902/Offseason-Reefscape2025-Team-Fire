// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DEADBAND = 0.15;
    }

    public static class SwerveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(15);
    }

    public static class FunnelIndexerConstants {
        public static final int LEFT_MOTOR_CAN_ID = 34;
        public static final int RIGHT_MOTOR_CAN_ID = 33;
        public static final int KICKER_MOTOR_CAN_ID = -1;

        public static final double FULL_SPEED = 0.5;
        public static final double HALF_SPEED = 0.1;
        public static final double REVERSE_SPEED = -0.1;
        public static final double STOP_SPEED = 0.0;

        public static final int SHALLOW_BEAM_BREAK_DIO = 2;
        public static final int DEEP_BEAM_BREAK_DIO = 3;

        public static final int MOTOR_CURRENT_LIMIT = 30;
    }

    public static class EndEffectorConstants {
        public static final int MOTOR_CAN_ID = 20;
        public static final int CORAL_BEAM_BREAK_PORT_ID = -1;
        public static final int ALGAE_PROXIMITY_SENSOR_PORT_ID = -1;
        public static final int MOTOR_STATOR_CURRENT_LIMIT = 40;
        public static final double INTAKE_SPEED = 1;
        public static final double OUTTAKE_SPEED = -1;
    }

    public static class VisionConstants {

    }

    public static class PathPlannerConstants {

    }

    public static class ArmConstants {
        //PID tuning mode
        public static final boolean kTuningMode = false;

        // CAN IDs
        public static final int kArmMotorCANID = 50;

        // Encoder Ports
        public static final int kArmEncoderPort = 5;

        // Physical Constants
        public static final double kArmGearing = 67.5;
        public static final double kArmMass = Units.lbsToKilograms(8);
        public static final double kArmLength = Units.inchesToMeters(13.386);   

        // Motion Constraints
        public static final double kArmMinAngle = -94; // Minimum angle for the arm
        public static final double kArmMaxAngle = 75;  

        // Current limits
        public static final double kStatorCurrentLimit = 50.0;
        public static final double kSupplyCurrentLimit = 50.0;

        // PID Constants
        public static double kArmP = 50;
        public static double kArmI = 0;
        public static double kArmD = 5;

        // Feedforward Constants
        public static double kArmS = 0.0; 
        public static double kArmG = 0.5; 
        public static double kArmV = 0.3; 
        public static double kArmA = 0.1; 

        // Arm Setpoints
        public static final double kArmZeroAngle = -94;
        public static final double kArmProcessorAngle = -50;

        public static final double kArmCoralLevel1Angle = 40.0; 
        public static final double kArmCoralLevel2Angle = 75.0;
        public static final double kArmCoralLevel3Angle = 75.0;
        public static final double kArmCoralLevel4Angle = 55.0;

        public static final double kArmAlgaeLowAngle = 0.0;
        public static final double kArmAlgaeHighAngle = 0.0;

        public static final double kArmBargeAngle = 50.0;
    }

    public static class ElevatorConstants {
        //PID tuning mode
        public static final boolean kTuningMode = false;

        // CAN IDs
        public static final int kElevatorLeaderCANID = 55;
        public static final int kElevatorFollowerCANID = 56;

        // Current Limits
        public static final double kElevatorStatorCurrentLimit = 50.0; 
        public static final double kElevatorSupplyCurrentLimit = 50.0; 

        // Physical Constants 
        public static final double kElevatorGearing = 4.875;
        public static final double kElevatorCarriageMass = Units.lbsToKilograms(24);
        public static final double kElevatorDrumRadius = Units.inchesToMeters(1);
        public static final double kElevatorMetersPerMotorRotation =
                (kElevatorDrumRadius * 2 * Math.PI) / kElevatorGearing;

        // Elevator Dimensions
        public static final double kElevatorHeightMeters = Units.inchesToMeters(42);
        public static final double kElevatorMinHeightMeters = Units.inchesToMeters(22.5);
        public static final double kElevatorMaxHeightMeters = Units.inchesToMeters(73.5);
        public static final double kElevatorCarriageHeightMeters = Units.inchesToMeters(18.5);

        // Motion Constraints
        public static final double kElevatorMaxVelocity =
                1.5 / ElevatorConstants.kElevatorMetersPerMotorRotation;
        public static final double kElevatorMaxAcceleration = 160.0;

        // PID Constants
        public static double kElevatorP = 95;
        public static double kElevatorI = 8;
        public static double kElevatorD = 8.5;

        // Elevator Gains
        public static double kElevatorS = 0.0;
        public static double kElevatorG = 2;
        public static double kElevatorV = 6.85 * kElevatorMetersPerMotorRotation;
        public static double kElevatorA = 0.04 * kElevatorMetersPerMotorRotation;

        // ===== Elevator Setpoints =====
        public static final double kElevatorProcessorHeight = Units.inchesToMeters(22.5);

        public static final double kElevatorCoralLevel1Height = Units.inchesToMeters(26.75);
        public static final double kElevatorCoralLevel2Height = Units.inchesToMeters(27.5);
        public static final double kElevatorCoralLevel3Height = Units.inchesToMeters(44);
        public static final double kElevatorCoralLevel4Height = Units.inchesToMeters(73.5);

        public static final double kElevatorAlgaeLowHeight = Units.inchesToMeters(28.5);
        public static final double kElevatorAlgaeHighHeight = Units.inchesToMeters(44.25);
        public static final double kElevatorBargeHeight = Units.inchesToMeters(73.5);
        // ==============================

        // ===== Control Parameters =====
        public static final double kElevatorTargetError = 0.005;
        public static final double kElevatorMotorResistance = 0.002; // Assume 2mOhm resistance for
        // voltage drop calculation
        // ==============================
    }
}
