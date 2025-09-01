// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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
  }
  public static class FunnelIndexerConstants {

  }

  public static class ArmConstants {
      // CAN IDs
      public static final int kArmMotorCANID = 50;

      // Encoder Ports
      public static final int kArmEncoderPort = 5;

      // Physical Constants
      public static final double kArmGearing = 67.5;
      public static final double kArmMass = Units.lbsToKilograms(8);
      public static final double kArmLength = Units.inchesToMeters(13.386);   

      // Motion Constraints
      public static final double kArmMinAngle = 0;
      public static final double kArmMaxAngle = 310;  

      // Current limits
      public static final double kStatorCurrentLimit = 30.0;
      public static final double kSupplyCurrentLimit = 30.0;

      // PID Constants
      public static final double kArmP = 30;
      public static final double kArmI = 0.0;
      public static final double kArmD = 0.1;

      // Feedforward Constants
      public static final double kArmS = 0.0; 
      public static final double kArmG = 0.5; 
      public static final double kArmV = 0.3; 
      public static final double kArmA = 0.1; 

      // Arm Setpoints
      public static final double kArmZeroAngle = 266.0;
      public static final double kArmProcessorAngle = 310.0;

      public static final double kArmCoralLevel1Angle = 40.0; 
      public static final double kArmCoralLevel2Angle = 75.0;
      public static final double kArmCoralLevel3Angle = 75.0;
      public static final double kArmCoralLevel4Angle = 55.0;

      public static final double kArmAlgaeLowAngle = 0.0;
      public static final double kArmAlgaeHighAngle = 0.0;

      public static final double kArmBargeAngle = 50.0;
    }
  public static class AlgaeCoralIndexerConstants {
    public static final int kMotorCANId = -1;
    public static final int kCoralBeamBreakPortId = -1;
    public static final int kAlgaeProximitySensorPortId = -1;
    public static final int kMotorStatorCurrentLimit = 40;
    public static final int kIntakeSpeed = 1;
    public static final int kOuttakeSpeed = -1;


  }

  public static class ElevatorConstants {
    // CAN IDs
    public static final int kElevatorLeaderCANID = 55;
    public static final int kElevatorFollowerCANID = 56;

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
    public static final double kElevatorP = 10;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.01;

    // Elevator Gains
    public static final double kElevatorS = 0.0;
    public static final double kElevatorG = 0.4;
    public static final double kElevatorV = 6.85 * kElevatorMetersPerMotorRotation;
    public static final double kElevatorA = 0.04 * kElevatorMetersPerMotorRotation;

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

  public static class PathPlannerConstants {

    }
  public static class VisionConstants {
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  }}