// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.15;
  }

  public static class SwerveConstants {
    public static final double MAX_SPEED = Units.feetToMeters(15);
  }

  public static class PhotonConstants {
    public static final String leftCamName = "left";
    public static final Transform3d leftCamToRobotTsf = 
      new Transform3d(0.207, 0.150, 0.567, new Rotation3d(Math.toRadians(0), Math.toRadians(30), Math.toRadians(-4.333)));
    public static final CameraProperties leftCamProp = 
      new CameraProperties(leftCamName, leftCamToRobotTsf, 640, 480, Rotation2d.fromDegrees(100), 30, 0.25, 0.08);

    public static final String rightCamName = "right";
    public static final Transform3d rightCamToRobotTsf = 
      new Transform3d(0.207, -0.150, 0.567, new Rotation3d(Math.toRadians(0), Math.toRadians(30), Math.toRadians(4.333)));
    public static final CameraProperties rightCamProp = 
      new CameraProperties(rightCamName, rightCamToRobotTsf, 640, 480, Rotation2d.fromDegrees(100), 30, 0.25, 0.08);

    public static final String middleCamName = "middle";
    public static final Transform3d middleCamToRobotTsf = 
      new Transform3d(0, 0, 0.35, new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));
    public static final CameraProperties middleCamProp =
      new CameraProperties(middleCamName, middleCamToRobotTsf, 640, 480, Rotation2d.fromDegrees(100), 30, 0.25, 0.08);
  }

  public static class LimelightConstants {
    public static final String leftCamName = "left";
    public static double kStdDevs = 0.800000;
  public static class FunnelIndexerConstants {

  }

  public static class AlgaeCoralIndexerConstants {
    public static final int kMotorCANId = -1;
    public static final int kCoralBeamBreakPortId = -1;
    public static final int kAlgaeProximitySensorPortId = -1;
    public static final int kMotorStatorCurrentLimit = 40;
    public static final int kIntakeSpeed = 1;
    public static final int kOuttakeSpeed = -1;


  }

  public static class ElevatorArmConstants {

  }

  public static class VisionConstants {
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  }
}