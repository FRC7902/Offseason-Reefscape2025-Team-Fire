// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }

  public static class SwerveConstants {

  }

  public static class FunnelIndexerConstants {

  }

  public static class AlgaeCoralIndexerConstants {
    public static final int motor_ID = 1; // Replace with actual motor ID
    public static final int algaebeam_ID = 2; // Replace with actual beam break ID
    public static final int coralbeam_ID = 3; // Replace with actual beam break ID

    public static final double take_algae = 12;
    public static final double outtake_algae = -12;
    public static final double take_coral = 12;
    public static final double outtake_coral = -12; 
  }

  public static class ElevatorArmConstants {

  }

  public static class VisionConstants {

  }

  public static class PathPlannerConstants {

  }
}
