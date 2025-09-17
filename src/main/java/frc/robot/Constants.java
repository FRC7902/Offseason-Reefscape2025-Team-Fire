// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.util.Units;
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

    public static class CoralIndexerConstants {

    }

    public static class AlgaeCoralIndexerConstants {

    }

    public static class ElevatorArmConstants {

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
        public static final Transform3d middleCamToRobotTsf = new Transform3d(0, 0, 0.35,
                new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));
        public static final CameraProperties middleCamProp = new CameraProperties(middleCamName, middleCamToRobotTsf,
                640, 480, Rotation2d.fromDegrees(100), 30, 0.25, 0.08);

        // Simulation constants
        public static final boolean enableVisionFieldSim = true;
        public static final boolean enableCameraPosChange = false;
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

    public static class PathPlannerConstants {

    }
}