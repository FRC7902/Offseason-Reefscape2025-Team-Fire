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
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignToReef extends Command {

    public enum ReefBranchSide {
        LEFT,
        RIGHT,
        CENTER
    }

    private final PIDController m_xController, m_yController, m_rotController;
    private final ReefBranchSide m_side;
    private Timer m_dontSeeTagTimer, m_stopTimer;
    private final SwerveSubsystem m_drivebase;
    private double m_tagID = -1;

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

        m_xController.setSetpoint(m_side == ReefBranchSide.CENTER ? VisionConstants.X_SETPOINT_FARTHER_REEF_ALIGNMENT : VisionConstants.X_SETPOINT_REEF_ALIGNMENT);
        m_xController.setTolerance(VisionConstants.X_TOLERANCE_REEF_ALIGNMENT);

        if (m_side == ReefBranchSide.RIGHT){
            m_yController.setSetpoint(VisionConstants.Y_SETPOINT_RIGHT_REEF_ALIGNMENT);
        }
        else if (m_side == ReefBranchSide.LEFT){
            m_yController.setSetpoint(VisionConstants.Y_SETPOINT_LEFT_REEF_ALIGNMENT);
        }
        else if (m_side == ReefBranchSide.CENTER){
            m_yController.setSetpoint(VisionConstants.Y_SETPOINT_CENTER_REEF_ALIGNMENT);
        }
        // m_yController.setSetpoint(
        //         m_side == ReefBranchSide.RIGHT ? VisionConstants.Y_SETPOINT_RIGHT_REEF_ALIGNMENT
        //                 : VisionConstants.Y_SETPOINT_LEFT_REEF_ALIGNMENT);
        m_yController.setTolerance(VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT);

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
            SmartDashboard.putNumber("AutoAlign - error x", m_xController.getError());
            SmartDashboard.putNumber("AutoAlign - error y", m_yController.getError());
            SmartDashboard.putNumber("AutoAlign - error rot", m_rotController.getError());

            SmartDashboard.putNumber("AutoAlign - xSpeed", xSpeed);
            SmartDashboard.putNumber("AutoAlign - ySpeed", ySpeed);
            SmartDashboard.putNumber("AutoAlign - rotValue", rotValue);

            double rotErrorScale = (90 - Math.min(90, Math.abs(m_rotController.getError()))) / 90;

            m_drivebase.drive(new Translation2d(xSpeed * rotErrorScale, ySpeed * rotErrorScale), rotValue, false);

            List<ElevatorSubsystem.ElevatorPosition> coralPositions = List.of(
                    ElevatorSubsystem.ElevatorPosition.CORAL_L1,
                    ElevatorSubsystem.ElevatorPosition.CORAL_L2,
                    ElevatorSubsystem.ElevatorPosition.CORAL_L3,
                    ElevatorSubsystem.ElevatorPosition.CORAL_L4
            );
            List<ElevatorSubsystem.ElevatorPosition> algaePositions = List.of(
                ElevatorSubsystem.ElevatorPosition.ALGAE_HIGH,
                ElevatorSubsystem.ElevatorPosition.ALGAE_LOW
            );
            ElevatorPosition currentPos = RobotContainer.m_elevatorSubsystem.getElevatorArmPositionEnum();
            if (coralPositions.contains(currentPos) &&
                    (RobotContainer.m_elevatorSubsystem.hasReachedSetpoint() && RobotContainer.m_armSubsystem.hasReachedAngle())
            ) {
                m_xController.setSetpoint(VisionConstants.X_SETPOINT_CLOSER_REEF_ALIGNMENT);
            }
            else if (algaePositions.contains(currentPos) &&
                    (RobotContainer.m_elevatorSubsystem.hasReachedSetpoint() && RobotContainer.m_armSubsystem.hasReachedAngle())) 
            {
                m_xController.setSetpoint(VisionConstants.X_SETPOINT_CLOSER_ALGAE_REEF_ALIGNMENT);
            }
            // Reset timer to mark 'not reached setpoint'.
            // Also reset timer if elevator/arm has not reached setpoint, since we want the auto-align setpoint
            // to update after reaching coral setpoint, so it can move closer after elevator/arm has extended fully
            if (!m_rotController.atSetpoint() ||
                    !m_yController.atSetpoint() ||
                    !m_xController.atSetpoint() ||
                    !(RobotContainer.m_elevatorSubsystem.hasReachedSetpoint() && RobotContainer.m_armSubsystem.hasReachedAngle())
            ) {
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
