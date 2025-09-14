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

        public static final int MOTOR_CURRENT_LIMIT = 20;
    }

    public static class EndEffectorConstants {
        public static final int MOTOR_CAN_ID = 20;
        public static final int CORAL_BEAM_BREAK_PORT_ID = 0;
        public static final int ALGAE_PROXIMITY_SENSOR_PORT_ID = -1;
        public static final int MOTOR_STATOR_CURRENT_LIMIT = 40;
        public static final double INTAKE_SPEED = 1;
        public static final double SLOW_INTAKE_SPEED = 0.1;
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
        public static final int kArmMotorCANID = 21;
        public static final int kArmCANID = 2;

        // Physical Constants
        public static final double kArmGearing = 67.5;
        public static final double kArmMass = Units.lbsToKilograms(8);
        public static final double kArmLength = Units.inchesToMeters(13.386);

        // Motion Constraints
        public static final double kArmMinAngle = -94; // Minimum angle for the arm
        public static final double kArmMaxAngle = 65;

        // Current limits
        public static final double kStatorCurrentLimit = 50.0;
        public static final double kSupplyCurrentLimit = 50.0;

        // PID Constants
        public static double kArmP = 60;
        public static double kArmI = 0.1;

        public static double kArmD = 0.01;

        // Feedforward Constants
        public static double kArmS = 0.0;

        public static double kArmG = 0.5;
        public static double kArmV = 0.3;
        public static double kArmA = 0.01;
        // SAFETIES
        public static double kbadARMPOS = 45;
        // Arm Setpoints
        public static final double kArmZeroAngle = -85;
        public static final double kArmProcessorAngle = -45;

        public static final double kArmCoralLevel1Angle = 25;
        //        public static final double kArmCoralLevel2Angle = 46.0;
//        public static final double kArmCoralLevel3Angle = 65;
//        public static final double kArmCoralLevel4Angle = 46;
        public static final double kArmCoralLevel2Angle = 58.0;
        public static final double kArmCoralLevel3Angle = 79.6;
        public static final double kArmCoralLevel4Angle = 62.57;

        public static final double kArmAlgaeLowAngle = 0.0;
        public static final double kArmAlgaeHighAngle = 0.0;

        public static final double kArmBargeAngle = 50.0;

        public static final double kArmTargetError = 3;
    }

    public static class ElevatorConstants {
        //PID tuning mode
        public static final boolean kTuningMode = false;

        // CAN IDs
        public static final int kElevatorLeaderCANID = 19;
        public static final int kElevatorFollowerCANID = 18;

        // Current Limits
        public static final double kElevatorStatorCurrentLimit = 80.0;
        public static final double kElevatorSupplyCurrentLimit = 50.0;

        // Physical Constants
        public static final double kElevatorGearing = 8.125;
        public static final double kElevatorCarriageMass = Units.lbsToKilograms(24);
        public static final double kElevatorDrumRadius = Units.inchesToMeters(1);
        public static final double kElevatorMetersPerMotorRotation =
                (kElevatorDrumRadius * 2 * Math.PI) / kElevatorGearing;

        // Elevator Dimensions
        public static final double kElevatorHeightMeters = Units.inchesToMeters(42);
        public static final double kElevatorMinHeightMeters = Units.inchesToMeters(0);
        public static final double kElevatorZeroThreshold = kElevatorMinHeightMeters + 0.01;
        public static final double kElevatorMaxHeightMeters = Units.inchesToMeters(49);
        public static final double kElevatorCarriageHeightMeters = Units.inchesToMeters(18.5);

        // SAFETY CONSTANT
        public static final double kAngleBad = -40;
        //        public static final double kElvPosBadMeters = Units.inchesToMeters(11.25);
        public static final double kElvPosBadMeters = 0.403;


        // Motion Constraints
        public static final double kElevatorMaxVelocity =
                2 / kElevatorMetersPerMotorRotation;
        // rotations per second
        public static final double kElevatorMaxAcceleration = 1600.0;

        // PID Constants
        public static double kElevatorP = 20;
        public static double kElevatorI = 0;
        public static double kElevatorD = 0;


        // Elevator Gains
        public static double kElevatorS = 0.0;
        public static double kElevatorG = 0.23;
        public static double kElevatorV = 0;//6.17 * kElevatorMetersPerMotorRotation;
        public static double kElevatorA = 0;//0.02;

        // ===== Elevator Setpoints =====
        public static final double kElevatorProcessorHeight = Units.inchesToMeters(22.5);

        public static final double kElevatorCoralLevel1Height = 0.250; // TODO
//        public static final double kElevatorCoralLevel2Height = Units.inchesToMeters(27.5);
//        public static final double kElevatorCoralLevel3Height = Units.inchesToMeters(30);
//        public static final double kElevatorCoralLevel4Height = Units.inchesToMeters(47);
        public static final double kElevatorCoralLevel2Height = 0.315;
        public static final double kElevatorCoralLevel3Height = 0.561;
        public static final double kElevatorCoralLevel4Height = 1.228;

        public static final double kElevatorAlgaeLowHeight = Units.inchesToMeters(28.5);
        public static final double kElevatorAlgaeHighHeight = Units.inchesToMeters(44.25);
        public static final double kElevatorBargeHeight = Units.inchesToMeters(47);
        // ==============================

        // ===== Control Parameters =====
        public static final double kElevatorTargetError = 0.01;
        public static final double kElevatorMotorResistance = 0.002; // Assume 2mOhm resistance for
        // voltage drop calculation
        // ==============================
    }
}
