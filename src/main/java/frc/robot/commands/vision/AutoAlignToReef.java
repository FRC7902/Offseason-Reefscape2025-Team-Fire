// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignToReef extends Command {

    public enum ReefBranchSide {
        LEFT,
        RIGHT
    }

    private final PIDController m_xController, m_yController, m_rotController;
    private ReefBranchSide m_side;
    private Timer m_dontSeeTagTimer, m_stopTimer;
    private SwerveSubsystem m_drivebase;
    private double m_tagID = -1;

    /**
     * Creates a new AutoAlignToReef.
     */
    public AutoAlignToReef(ReefBranchSide side) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_xController = new PIDController(Constants.VisionConstants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
        m_yController = new PIDController(Constants.VisionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horizontal movement
        m_rotController = new PIDController(Constants.VisionConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
        m_side = side;
        this.m_drivebase = RobotContainer.m_swerveSubsystem;
        addRequirements(RobotContainer.m_swerveSubsystem);

        SmartDashboard.putNumber("AutoAlign - error x", 0);
        SmartDashboard.putNumber("AutoAlign - error y", 0);
        SmartDashboard.putNumber("AutoAlign - error rot", 0);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.m_stopTimer = new Timer();
        this.m_stopTimer.start();
        this.m_dontSeeTagTimer = new Timer();
        this.m_dontSeeTagTimer.start();

        m_rotController.setSetpoint(Constants.VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT);
        m_rotController.setTolerance(Constants.VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

        m_xController.setSetpoint(Constants.VisionConstants.X_SETPOINT_REEF_ALIGNMENT);
        m_xController.setTolerance(Constants.VisionConstants.X_TOLERANCE_REEF_ALIGNMENT);

        m_yController.setSetpoint(
                m_side == ReefBranchSide.RIGHT ?
                        Constants.VisionConstants.Y_SETPOINT_REEF_ALIGNMENT :
                        -Constants.VisionConstants.Y_SETPOINT_REEF_ALIGNMENT
        );
        m_yController.setTolerance(Constants.VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT);

        m_tagID = LimelightHelpers.getFiducialID("");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == m_tagID) {
            this.m_dontSeeTagTimer.reset();

            double[] positions = LimelightHelpers.getBotPose_TargetSpace("");

            double xSpeed = -m_xController.calculate(positions[2]);

            double ySpeed = m_yController.calculate(positions[0]);

            double rotValue = -m_rotController.calculate(positions[4]);

            // Error values
            SmartDashboard.putNumber("AutoAlign - error x", positions[2] - m_xController.getSetpoint());
            SmartDashboard.putNumber("AutoAlign - error y", positions[0] - m_yController.getSetpoint());
            SmartDashboard.putNumber("AutoAlign - error rot", positions[4] - m_rotController.getSetpoint());

//            SmartDashboard.putNumber("AutoAlign - xSpeed", xSpeed);
//            SmartDashboard.putNumber("AutoAlign - ySpeed", ySpeed);
//            SmartDashboard.putNumber("AutoAlign - rotValue", rotValue);

            m_drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

            if (!m_rotController.atSetpoint() ||
                    !m_yController.atSetpoint() ||
                    !m_xController.atSetpoint()) {
                m_stopTimer.reset();
            }
        } else {
            m_drivebase.drive(new Translation2d(), 0, false);
        }

        SmartDashboard.putNumber("poseValidTimer", m_stopTimer.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivebase.drive(new Translation2d(), 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.m_dontSeeTagTimer.hasElapsed(Constants.VisionConstants.DONT_SEE_TAG_WAIT_TIME) ||
                m_stopTimer.hasElapsed(Constants.VisionConstants.POSE_VALIDATION_TIME);
    }
}
