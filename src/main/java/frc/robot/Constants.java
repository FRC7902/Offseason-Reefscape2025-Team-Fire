// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.CameraProperties;

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
        public static final double MIN_TRANSLATION_SPEED_SCALE = 0.25; // Minimum speed scaling factor for joystick input
        public static final double MIN_ROTATION_SPEED_SCALE = 0.25; // Minimum speed scaling factor for joystick input
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
        public static final double MAX_HEIGHT_METERS = 1.22;
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
        public static final double L1_HEIGHT_METERS = ElevatorConstants.MAX_HEIGHT_METERS * 0.02195122 + 0.10;
        public static final double L2_HEIGHT_METERS = ElevatorConstants.MAX_HEIGHT_METERS * 0.21219512 + 0.10;
        public static final double L3_HEIGHT_METERS = ElevatorConstants.MAX_HEIGHT_METERS * 0.41463415 + 0.10;
        public static final double L4_HEIGHT_METERS = ElevatorConstants.MAX_HEIGHT_METERS;

        public static final double LOW_ALGAE_HEIGHT_METERS = ElevatorConstants.MAX_HEIGHT_METERS * 0.12195122;
        public static final double HIGH_ALGAE_HEIGHT_METERS = ElevatorConstants.MAX_HEIGHT_METERS * 0.41463415 + 0.025;
        public static final double BARGE_HEIGHT_METERS = ElevatorConstants.MAX_HEIGHT_METERS;
        // ==============================

        // ===== Control Parameters =====
        public static final double TARGET_ERROR = 0.01;
        public static final double MOTOR_RESISTANCE = 0.002; // Assume 2mOhm resistance for
        // voltage drop calculation
        // ==============================
    }

        public static class VisionConstants {
        // Contains the stored position of each April Tag on the field. This varies
        // between seasons.
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeWelded);
        public static final double xOffset = 0.8;
        public static final double aprilTagOffset = 0.1;

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public static double kPXClose = 3.4;
        public static double kIXClose = 0;
        public static double kDXClose = 0.01;

        public static double kPYClose = 5;
        public static double kIYClose = 0;
        public static double kDYClose = 0.2;

        public static double kPXFar = 3.6;
        public static double kIXFar = 0;
        public static double kDXFar = 0.05;

        public static double kPYFar = 5;
        public static double kIYFar = 0;
        public static double kDYFar = 0.2;

        public static double kPTheta = 3.6;
        public static double kITheta = 0;
        public static double kDTheta = 0.05;

        public static double kPIDDifferenceConstantX = 3;
        public static double kPIDDifferenceConstantY = 3;

        public static final Constraints kXConstraints = new Constraints(20, 20);
        public static final Constraints kYConstraints = new Constraints(20, 20);
        public static final Constraints kOmegaConstraints = new Constraints(20, 20);

        public enum WAYPOINT_LOCATIONS {
            RED_FRONT,
            RED_FRONT_LEFT,
            RED_BACK_LEFT,
            RED_BACK,
            RED_FRONT_RIGHT,
            RED_BACK_RIGHT,
            BLUE_FRONT,
            BLUE_FRONT_LEFT,
            BLUE_FRONT_RIGHT,
            BLUE_BACK_LEFT,
            BLUE_BACK,
            BLUE_BACK_RIGHT
        }

        public static final java.util.Map<WAYPOINT_LOCATIONS, Pose2d> WAYPOINTS = java.util.Map.ofEntries(
                Map.entry(WAYPOINT_LOCATIONS.RED_FRONT, new Pose2d(14.24, 3.99, new Rotation2d(Math.toRadians(0)))),
                Map.entry(WAYPOINT_LOCATIONS.RED_FRONT_LEFT, new Pose2d(13.75, 3.03, new Rotation2d(Math.toRadians(300)))),
                Map.entry(WAYPOINT_LOCATIONS.RED_BACK_LEFT, new Pose2d(12.39, 2.88, new Rotation2d(Math.toRadians(240)))),
                Map.entry(WAYPOINT_LOCATIONS.RED_BACK, new Pose2d(11.87, 3.99, new Rotation2d(Math.toRadians(180)))),
                Map.entry(WAYPOINT_LOCATIONS.RED_FRONT_RIGHT, new Pose2d(13.65, 5, new Rotation2d(Math.toRadians(60)))),
                Map.entry(WAYPOINT_LOCATIONS.RED_BACK_RIGHT, new Pose2d(12.50, 5, new Rotation2d(Math.toRadians(120)))),
                Map.entry(WAYPOINT_LOCATIONS.BLUE_FRONT, new Pose2d(3.36, 4.02, new Rotation2d(Math.toRadians(0)))),
                Map.entry(WAYPOINT_LOCATIONS.BLUE_FRONT_LEFT, new Pose2d(3.94, 3.98, new Rotation2d(Math.toRadians(300)))),
                Map.entry(WAYPOINT_LOCATIONS.BLUE_FRONT_RIGHT, new Pose2d(3.93, 3.06, new Rotation2d(Math.toRadians(60)))),
                Map.entry(WAYPOINT_LOCATIONS.BLUE_BACK_LEFT, new Pose2d(5.06, 4.99, new Rotation2d(Math.toRadians(240)))),
                Map.entry(WAYPOINT_LOCATIONS.BLUE_BACK, new Pose2d(5.59, 4.01, new Rotation2d(Math.toRadians(180)))),
                Map.entry(WAYPOINT_LOCATIONS.BLUE_BACK_RIGHT, new Pose2d(5.02, 3.07, new Rotation2d(Math.toRadians(120)))));
    }


    public static class PhotonConstants {
        public static final String leftCamName = "left";
        // public static final Transform3d leftCamToRobotTsf = new Transform3d(0.207, 0.150, 0.567,
        //         new Rotation3d(Math.toRadians(0), Math.toRadians(30), Math.toRadians(-4.333)));
        // public static final CameraProperties leftCamProp = new CameraProperties(leftCamName, leftCamToRobotTsf, 640,
        //         480, Rotation2d.fromDegrees(100), 30, 0.25, 0.08);

        // public static final String rightCamName = "right";
        // public static final Transform3d rightCamToRobotTsf = new Transform3d(0.207, -0.150, 0.567,
        //         new Rotation3d(Math.toRadians(0), Math.toRadians(30), Math.toRadians(4.333)));
        // public static final CameraProperties rightCamProp = new CameraProperties(rightCamName, rightCamToRobotTsf, 640,
        //         480, Rotation2d.fromDegrees(100), 30, 0.25, 0.08);

        public static final String middleCamName = "middle";
        public static final Transform3d middleCamToRobotTsf = new Transform3d(-0.271, 0.320, 0.198,
                new Rotation3d(Math.toRadians(0), Math.toRadians(15), Math.toRadians(22)));
        public static final CameraProperties middleCamProp = new CameraProperties(middleCamName, middleCamToRobotTsf,
                640, 480, Rotation2d.fromDegrees(100), 30, 0.25, 0.08);

        // Simulation constants
        public static final boolean enableVisionFieldSim = true;
        public static final boolean enableCameraPosChange = false;
    }
}
