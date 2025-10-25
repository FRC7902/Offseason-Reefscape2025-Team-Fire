// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignToReef extends Command {

    public enum ReefBranchSide {
        LEFT,
        RIGHT
    }

    private PIDController m_xController, m_yController, m_rotController;
    private final ReefBranchSide m_side;
    private Timer m_dontSeeTagTimer, m_stopTimer;
    private final SwerveSubsystem m_drivebase;
    private double m_tagID = -1;

    double X_P;
    double X_I;
    double X_D;

    double Y_P;
    double Y_I;
    double Y_D;

    double ROT_P;
    double ROT_I;
    double ROT_D;

    /**
     * Creates a new AutoAlignToReef.
     */
    public AutoAlignToReef(ReefBranchSide side) {
        m_xController = new PIDController(VisionConstants.X_REEF_ALIGNMENT_P, 0.0, 0); // Vertical movement
        m_yController = new PIDController(VisionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0); // Horizontal movement
        m_rotController = new PIDController(VisionConstants.ROT_REEF_ALIGNMENT_P, 0, 0); // Rotation

        m_side = side;

        this.m_drivebase = RobotContainer.m_swerveSubsystem;
        addRequirements(RobotContainer.m_swerveSubsystem);

        SmartDashboard.putNumber("AutoAlign - error x", 0);
        SmartDashboard.putNumber("AutoAlign - error y", 0);
        SmartDashboard.putNumber("AutoAlign - error rot", 0);

        SmartDashboard.putNumber("AutoAlign - X_P", X_P);
        SmartDashboard.putNumber("AutoAlign - X_I", X_I);
        SmartDashboard.putNumber("AutoAlign - X_D", X_D);

        SmartDashboard.putNumber("AutoAlign - Y_P", Y_P);
        SmartDashboard.putNumber("AutoAlign - Y_I", Y_I);
        SmartDashboard.putNumber("AutoAlign - Y_D", Y_D);

        SmartDashboard.putNumber("AutoAlign - ROT_P", ROT_P);
        SmartDashboard.putNumber("AutoAlign - ROT_I", ROT_I);
        SmartDashboard.putNumber("AutoAlign - ROT_D", ROT_D);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.m_stopTimer = new Timer();
        this.m_stopTimer.start();
        this.m_dontSeeTagTimer = new Timer();
        this.m_dontSeeTagTimer.start();

        m_rotController.setSetpoint(VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT);
        m_rotController.setTolerance(VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

        m_xController.setSetpoint(VisionConstants.X_SETPOINT_REEF_ALIGNMENT);
        m_xController.setTolerance(VisionConstants.X_TOLERANCE_REEF_ALIGNMENT);

        m_yController.setSetpoint(
                m_side == ReefBranchSide.RIGHT ? VisionConstants.Y_SETPOINT_REEF_ALIGNMENT
                        : -VisionConstants.Y_SETPOINT_REEF_ALIGNMENT);
        m_yController.setTolerance(VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT);

        m_tagID = LimelightHelpers.getFiducialID("");

        X_P = SmartDashboard.getNumber("AutoAlign - X_P",X_P);
        X_I = SmartDashboard.getNumber("AutoAlign - X_I",X_I);
        X_D = SmartDashboard.getNumber("AutoAlign - X_D",X_D);

        Y_P = SmartDashboard.getNumber("AutoAlign - Y_P",Y_P);
        Y_I = SmartDashboard.getNumber("AutoAlign - Y_I",Y_I);
        Y_D = SmartDashboard.getNumber("AutoAlign - Y_D",Y_D);

        ROT_P = SmartDashboard.getNumber("AutoAlign - ROT_P",ROT_P);
        ROT_I = SmartDashboard.getNumber("AutoAlign - ROT_I",ROT_I);
        ROT_D = SmartDashboard.getNumber("AutoAlign - ROT_D",ROT_D);

        m_xController = new PIDController(X_P,X_I,X_D); // Vertical movement
        m_yController = new PIDController(Y_P, Y_I,Y_D); // Horizontal movement
        m_rotController = new PIDController(ROT_P,ROT_I, ROT_D); // Rotation
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
            SmartDashboard.putNumber("AutoAlign - error x", m_xController.getError());
            SmartDashboard.putNumber("AutoAlign - error y", m_yController.getError());
            SmartDashboard.putNumber("AutoAlign - error rot", m_rotController.getError());

            SmartDashboard.putNumber("AutoAlign - xSpeed", xSpeed);
            SmartDashboard.putNumber("AutoAlign - ySpeed", ySpeed);
            SmartDashboard.putNumber("AutoAlign - rotValue", rotValue);

            double rotErrorScale = (90 - Math.min(90, Math.abs(m_rotController.getError()))) / 90;

            m_drivebase.drive(new Translation2d(xSpeed * rotErrorScale, ySpeed * rotErrorScale), rotValue, false);

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
        return this.m_dontSeeTagTimer.hasElapsed(VisionConstants.DONT_SEE_TAG_WAIT_TIME) ||
                m_stopTimer.hasElapsed(VisionConstants.POSE_VALIDATION_TIME);
    }
}
