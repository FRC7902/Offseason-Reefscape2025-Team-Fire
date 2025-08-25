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
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.15;
  }

  public static class SwerveConstants {
    public static final double MAX_SPEED = Units.feetToMeters(15);
  }

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

  }

  public static class PathPlannerConstants {

  }
}