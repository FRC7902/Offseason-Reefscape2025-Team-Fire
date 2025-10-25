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
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double DEADBAND = 0.15;
    }

    public static class SwerveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(30);

        // Speed scaling factors, should be between 0 and 1
        public static final double MIN_TRANSLATION_SPEED_SCALE = 0.175; // Minimum speed scaling factor for joystick input
        public static final double MIN_ROTATION_SPEED_SCALE = 0.25; // Minimum speed scaling factor for joystick input

        public static final double FAST_DRIVE_RAMP_RATE = 0.15;
    }

    public static class PathPlanner {
        public static final double kPDrive = 1.95;
        public static final double kIDrive = 0;
        public static final double kDDrive = 0.01;

        public static final double kPAngle = 2.6;
        public static final double kIAngle = 0;
        public static final double kDAngle = 0.01;

    }


    public static class FunnelIndexerConstants {
        public static final int LEFT_MOTOR_CAN_ID = 34;
        public static final int RIGHT_MOTOR_CAN_ID = 33;
        public static final int KICKER_MOTOR_CAN_ID = -1;

        public static final double FULL_SPEED = 0.5;
        public static final double HALF_SPEED = 0.1;
        public static final double OUTTAKE_SPEED = 0.25;
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

        public static final int MOTOR_STATOR_CURRENT_LIMIT = 80;
        public static final int MOTOR_SUPPLY_CURRENT_LIMIT = 40;

        public static final double INTAKE_SPEED = 1;

        public static final double CORAL_HOLD_SPEED = 0.1;
        public static final double ALGAE_HOLD_SPEED = 1.0;

        public static final double OUTTAKE_SPEED = -1;

        public static final double ALGAE_INTAKE_STALL_DETECTION_CURRENT_LOW = 11;
        public static final double ALGAE_INTAKE_STALL_DETECTION_CURRENT_HIGH = 13;
    }

    public static class VisionConstants {
        public static final double X_REEF_ALIGNMENT_P = 3.3;
	    public static final double Y_REEF_ALIGNMENT_P = 3.3;
	    public static final double ROT_REEF_ALIGNMENT_P = 0.058;

        public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
	    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
	    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.75;  // Vertical pose
	    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
	    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.00;  // Horizontal pose
	    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

        public static final double DONT_SEE_TAG_WAIT_TIME = 1;
	    public static final double POSE_VALIDATION_TIME = 0.3;

        public static final double LOCALIZE_DISTANCE_THRESHOLD = 3;
        public static final double LOCALIZE_AMBIGUITY_THRESHOLD = 0.7;
    }

    public static class PathPlannerConstants {

    }

    public static class ArmConstants {
        //PID tuning mode
        public static final boolean TUNING_MODE_ENABLED = false;

        // CAN IDs
        public static final int MOTOR_CAN_ID = 21;
        public static final int ENCODER_CAN_ID = 2;

        // Physical Constants
        public static final double GEARING = 67.5;
        public static final double MASS_KG = Units.lbsToKilograms(8);
        public static final double LENGTH_METERS = Units.inchesToMeters(13.386);

        // Motion Constraints
        public static final double MIN_ANGLE_DEGREES = -94; // Minimum angle for the arm
        public static final double MAX_ANGLE_DEGREES = 65;

        // Current limits
        public static final double STATOR_CURRENT_LIMIT = 50.0;
        public static final double SUPPLY_CURRENT_LIMIT = 50.0;

        // PID Constants
        public static double PID_P = 60;
        public static double PID_I = 0.1;
        public static double PID_D = 0.01;

        // Feedforward Constants
        public static double FF_S = 0.0;
        public static double FF_G = 0.5;
        public static double FF_V = 0.3;
        public static double FF_A = 0.01;

        // SAFETIES
        public static final double SAFETY_ANGLE_DOWNWARD_DEGREES = 45;
        public static final double SAFETY_ANGLE_UPWARD_DEGREES = -40;

        // Arm Setpoints
        public static final double ZERO_ANGLE_DEGREES = -90;
        public static final double REST_ANGLE_DEGREES = 55;
        public static final double PROCESSOR_ANGLE_DEGREES = -45.0;

        public static final double L1_ANGLE_DEGREES = 45.0;
        public static final double L2_ANGLE_DEGREES = 56.0;
        public static final double L3_ANGLE_DEGREES = 75.0;
        public static final double L4_ANGLE_DEGREES = 56.0;

        public static final double LOW_ALGAE_ANGLE_DEGREES = 0.0;
        public static final double HIGH_ALGAE_ANGLE_DEGREES = 0.0;

        public static final double BARGE_ANGLE_DEGREES = 50.0;

        public static final double TARGET_ERROR = 3;
    }

    public static class ElevatorConstants {
        //PID tuning mode
        public static final boolean TUNING_MODE_ENABLED = false;

        // CAN IDs
        public static final int LEADER_MOTOR_CAN_ID = 19;
        public static final int FOLLOWER_MOTOR_CAN_ID = 18;

        // Current Limits
        public static final double STATOR_CURRENT_LIMIT = 80.0;
        public static final double SUPPLY_CURRENT_LIMIT = 50.0;

        // Physical Constants
        public static final double GEARING = 8.125;
        public static final double CARRIAGE_MASS = Units.lbsToKilograms(24);
        public static final double DRUM_RADIUS = Units.inchesToMeters(1);
        public static final double METERS_PER_MOTOR_ROTATION =
                (DRUM_RADIUS * 2 * Math.PI) / GEARING;

        // Elevator Dimensions
        public static final double HEIGHT_METERS = Units.inchesToMeters(42);
        public static final double MIN_HEIGHT_METERS = Units.inchesToMeters(0);
        public static final double ZERO_THRESHOLD = MIN_HEIGHT_METERS + 0.01;
        public static final double MAX_HEIGHT_METERS = 1.27;
        public static final double CARRIAGE_HEIGHT_METERS = Units.inchesToMeters(18.5);

        // Elevator safety
        public static final double SAFETY_POSITION_METERS = 0.403;


        // Motion Constraints
        public static final double MAX_VELOCITY =
                2 / METERS_PER_MOTOR_ROTATION;
        // rotations per second
        public static final double MAX_ACCELERATION = 1600.0;

        // PID Constants
        public static double PID_P = 20;
        public static double PID_I = 0;
        public static double PID_D = 0;


        // Elevator Gains
        public static double FF_S = 0.0;
        public static double FF_G = 0.23;
        public static double FF_V = 0;//6.17 * kElevatorMetersPerMotorRotation;
        public static double FF_A = 0;//0.02;

        // ===== Elevator Setpoints =====
        public static final double PROCESSOR_HEIGHT_METERS = ElevatorConstants.MIN_HEIGHT_METERS;

        public static final double REST_HEIGHT_METERS = ElevatorConstants.MIN_HEIGHT_METERS + 0.05;
        public static final double L1_HEIGHT_METERS = 0.12678049;
        public static final double L2_HEIGHT_METERS = 0.15887805;
        public static final double L3_HEIGHT_METERS = 0.40585366;
        public static final double L4_HEIGHT_METERS = 1.22;

        public static final double LOW_ALGAE_HEIGHT_METERS = 0.14878049;
        public static final double HIGH_ALGAE_HEIGHT_METERS = 0.53085366;
        public static final double BARGE_HEIGHT_METERS = ElevatorConstants.MAX_HEIGHT_METERS;
        // ==============================

        // ===== Control Parameters =====
        public static final double TARGET_ERROR = 0.01;
        public static final double MOTOR_RESISTANCE = 0.002; // Assume 2mOhm resistance for
        // voltage drop calculation
        // ==============================
    }
}
