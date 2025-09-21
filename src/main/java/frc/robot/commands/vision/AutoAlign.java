// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.PhotonSubsystem;
import frc.robot.subsystems.vision.ReefSide;
import swervelib.SwerveController;
import swervelib.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final SwerveDrive m_swerveDrive;
  private final SwerveController m_swerveController;

  private final PhotonSubsystem m_camera;
  private final ProfiledPIDController m_xController;
  private final ProfiledPIDController m_yController;

  private Pose2d robotPose;
  private Pose2d aprilTagPose;

  private final double reefOffset;
  private double aprilTagRotation;

  private Transform2d aprilTagDistanceToRobot;

  // private final HolonomicDriveController m_drivePID;

  /** Creates a new ArcadeDriveCommand. */
  public AutoAlign(SwerveSubsystem m_swerveSubsystem, PhotonSubsystem m_camera, int reefSide) {
    this.m_swerveSubsystem = m_swerveSubsystem;
    this.m_camera = m_camera;

    m_swerveDrive = m_swerveSubsystem.getSwerveDrive();
    m_swerveController = m_swerveDrive.getSwerveController();

    m_xController = new ProfiledPIDController(VisionConstants.kPXClose, VisionConstants.kIXClose, VisionConstants.kDXClose, VisionConstants.kXConstraints);
    m_yController = new ProfiledPIDController(VisionConstants.kPYClose, VisionConstants.kIYClose, VisionConstants.kDYClose, VisionConstants.kYConstraints);

    switch (reefSide) {
      case 0:
        reefOffset = -VisionConstants.aprilTagOffset;
        break;
      case 1:
        reefOffset = VisionConstants.aprilTagOffset;;
        break;
      default:
        reefOffset = 0;
    }

    // SmartDashboard.putNumber("KPX Close", VisionConstants.kPXClose);
    // SmartDashboard.putNumber("KIX Close", VisionConstants.kIXClose);
    // SmartDashboard.putNumber("KDX Close", VisionConstants.kDXClose);

    // SmartDashboard.putNumber("KPY Close", VisionConstants.kPYClose);
    // SmartDashboard.putNumber("KIY Close", VisionConstants.kIYClose);
    // SmartDashboard.putNumber("KDY Close", VisionConstants.kDYClose);

    // SmartDashboard.putNumber("KPX Far", VisionConstants.kPXFar);
    // SmartDashboard.putNumber("KIX Far", VisionConstants.kIXFar);
    // SmartDashboard.putNumber("KDX Far", VisionConstants.kDXFar);

    // SmartDashboard.putNumber("KPY Far", VisionConstants.kPYFar);
    // SmartDashboard.putNumber("KIY Far", VisionConstants.kIYFar);
    // SmartDashboard.putNumber("KDY Far", VisionConstants.kDYFar);

    // SmartDashboard.putNumber("PID X DIFF", VisionConstants.kPIDDifferenceConstantX);
    // SmartDashboard.putNumber("PID Y DIFF", VisionConstants.kPIDDifferenceConstantY);

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    robotPose = m_swerveDrive.getPose();
    aprilTagPose = robotPose.nearest(m_camera.getReefPoses());

    m_xController.setGoal(0.0);
    m_yController.setGoal(0.0);

    m_xController.setTolerance(0.01);
    m_yController.setTolerance(0.01);

    aprilTagRotation = MathUtil.angleModulus(aprilTagPose.getRotation().getRadians());

    double multiplier = Math.round(aprilTagRotation / Math.abs(aprilTagRotation));

    if (aprilTagRotation == 0) {
      aprilTagRotation = Math.PI;
    }
    
    else {
      aprilTagRotation -= (Math.PI * multiplier);
    }

    if (m_camera.getTagTX() >= VisionConstants.kPIDDifferenceConstantX) {
      m_xController.setPID(VisionConstants.kPXFar, VisionConstants.kIXFar, VisionConstants.kDXFar);
    }

    else {
      m_xController.setPID(VisionConstants.kPXClose, VisionConstants.kIXClose, VisionConstants.kDXClose);
    }

    if (m_camera.getTagTY() >= VisionConstants.kPIDDifferenceConstantY) {
      m_yController.setPID(VisionConstants.kPYFar, VisionConstants.kIYFar, VisionConstants.kDYFar);
    }

    else {
      m_yController.setPID(VisionConstants.kPYClose, VisionConstants.kIYClose, VisionConstants.kDYClose);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    robotPose = m_swerveDrive.getPose();
    aprilTagDistanceToRobot = aprilTagPose.minus(robotPose);

    double robotRotation = m_swerveDrive.getPose().getRotation().getRadians();

    double xSpeed = m_xController.calculate(aprilTagDistanceToRobot.getX() - 0.5);
    double ySpeed = m_yController.calculate(aprilTagDistanceToRobot.getY() + reefOffset);
    double omegaSpeed = m_swerveController.headingCalculate(robotRotation, aprilTagRotation);

    ChassisSpeeds targetSpeed = ChassisSpeeds.discretize(new ChassisSpeeds(-xSpeed, -ySpeed, omegaSpeed), 0.02);

    m_swerveDrive.drive(targetSpeed);

    // SmartDashboard.putNumber("X error", m_xController.getPositionError());
    // SmartDashboard.putNumber("Y error", m_yController.getPositionError());
    // SmartDashboard.putNumber("Robot Rotation", robotRotation);
    // SmartDashboard.putNumber("April Tag Rotation", aprilTagRotation);

    // SmartDashboard.putNumber("Y Speed", ySpeed);
    // SmartDashboard.putNumber("Omega Speed", ySpeed);



  }

  void updatePID() {


    VisionConstants.kPXClose = SmartDashboard.getNumber("KPX Close", VisionConstants.kPXClose);
    VisionConstants.kIXClose = SmartDashboard.getNumber("KIX Close", VisionConstants.kIXClose);
    VisionConstants.kDXClose = SmartDashboard.getNumber("KDX Close", VisionConstants.kDXClose);

    VisionConstants.kPYClose = SmartDashboard.getNumber("KPY Close", VisionConstants.kPYClose);
    VisionConstants.kIYClose = SmartDashboard.getNumber("KIY Close", VisionConstants.kIYClose);
    VisionConstants.kDYClose = SmartDashboard.getNumber("KDY Close", VisionConstants.kDYClose);

    VisionConstants.kPXFar = SmartDashboard.getNumber("KPX Far", VisionConstants.kPXFar);
    VisionConstants.kIXFar = SmartDashboard.getNumber("KIX Far", VisionConstants.kIXFar);
    VisionConstants.kDXFar = SmartDashboard.getNumber("KDX Far", VisionConstants.kDXFar);

    VisionConstants.kPYFar = SmartDashboard.getNumber("KPY Far", VisionConstants.kPYFar);
    VisionConstants.kIYFar = SmartDashboard.getNumber("KIY Far", VisionConstants.kIYFar);
    VisionConstants.kDYFar = SmartDashboard.getNumber("KDY Far", VisionConstants.kDYFar);

    VisionConstants.kPIDDifferenceConstantX = SmartDashboard.getNumber("PID X DIFF", VisionConstants.kPIDDifferenceConstantX);
    VisionConstants.kPIDDifferenceConstantY = SmartDashboard.getNumber("PID Y DIFF", VisionConstants.kPIDDifferenceConstantY);

    SmartDashboard.putNumber("APRIL TAG TX", m_camera.getTagTX());
    SmartDashboard.putNumber("APRIL TAG TY", m_camera.getTagTY());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_xController.atGoal() && m_yController.atGoal();
  }
}