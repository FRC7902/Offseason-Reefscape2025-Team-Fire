package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

public class LimelightSubsystem extends SubsystemBase {
    
    private final SwerveSubsystem m_swerveSubsystem;
    private final SwerveDrive m_swerveDrive;
    private final String camera;

    public LimelightSubsystem(SwerveSubsystem m_swerveSubsystem, String camera) {
        this.m_swerveSubsystem = m_swerveSubsystem;
        m_swerveDrive = m_swerveSubsystem.getSwerveDrive();
        this.camera = camera;

        LimelightHelpers.SetIMUMode(camera, 1);
    }

    public double getTagTX() {
        return LimelightHelpers.getTX(camera);
    }

    public double getTagTY() {
        return LimelightHelpers.getTY(camera);
    }

    public double getTagArea() {
        return LimelightHelpers.getTA(camera);
    }

    public boolean cameraSeesAprilTag() {
        return LimelightHelpers.getTV(camera);
    }

    public boolean shouldAcceptUpdate(LimelightHelpers.PoseEstimate limelightMeasurement) {
        double rotationSpeed = Math
                .abs(m_swerveDrive.getRobotVelocity().omegaRadiansPerSecond);

        double estimatedDistance = limelightMeasurement.avgTagDist;

        int tagsDetected = limelightMeasurement.tagCount;

        if (rotationSpeed >= Math.PI * 2)
            return false;
        else if (tagsDetected <= 0) 
            return false;
        else 
            return true;
    }

    public void updateOdometryWithMegaTag1() {
        if (cameraSeesAprilTag()) {
            LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(camera);

            double rotationSpeed = Math
                    .abs(m_swerveDrive.getRobotVelocity().omegaRadiansPerSecond);
            boolean shouldRejectUpdate = rotationSpeed < 6.28319 && getTagArea() > 50; // 360 degrees

            if (shouldAcceptUpdate(limelightMeasurement)) {
                m_swerveDrive.addVisionMeasurement(
                        limelightMeasurement.pose,
                        limelightMeasurement.timestampSeconds,
                        VecBuilder.fill(.5, .5, 9999999));
            }

        }
    }

    public void updateOdometryWithMegaTag2() {
        if (cameraSeesAprilTag()) {
            LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(camera);

            if (limelightMeasurement != null) {

                double rotationSpeed = Math.abs(m_swerveDrive.getRobotVelocity().omegaRadiansPerSecond);
                boolean shouldRejectUpdate = rotationSpeed < 6.28319 && getTagArea() > 50; // 360 degrees

                m_swerveDrive.setVisionMeasurementStdDevs(
                        VecBuilder.fill(LimelightConstants.kStdDevs, LimelightConstants.kStdDevs, 9999999));

                Pose2d cameraPose = limelightMeasurement.pose;

                if (shouldAcceptUpdate(limelightMeasurement)) {
                    m_swerveDrive.addVisionMeasurement(
                            limelightMeasurement.pose,
                            limelightMeasurement.timestampSeconds);
                    // RobotContainer.m_swerveSubsystem.getSwerveDrive().updateOdometry();
                }
            }
        }

    }

    @Override
    public void periodic() {
        m_swerveDrive.updateOdometry();
        LimelightHelpers.SetRobotOrientation(camera,
            m_swerveDrive.getPose().getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
        updateOdometryWithMegaTag2();
    }

}
