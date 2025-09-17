package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import swervelib.SwerveDrive;
import frc.robot.Constants.VisionConstants;

public class PhotonSubsystem extends SubsystemBase {

    private final CameraProperties m_camProperties;
    private final PhotonCamera m_camera;
    private final SwerveDrive m_swerveDrive;
    private final PhotonSim m_photonSim;
    private final PhotonPoseEstimator m_poseEstimator;
    private final String cameraName;
    private Transform3d camToRobotTsf;
    private DetectedTags detectionStatus = DetectedTags.NONE;

    private Matrix<N3, N1> curStdDevs;
    private double aprilTagRot = 0;
    private double aprilTagTx = 0;
    private double aprilTagTy = 0;
    private double aprilTagArea = 0;
    private int aprilTagID = 0;
    private int detectedTagsCount = 0;


    /**
     * Creates the PhotonVision camera object. This is where all the camera data is processed.
     * If using a real camera, ensure the camera name matches the name set in PhotonVision. Otherwise, it will not detect the camera.
     * @param m_camProperties The {@code CameraProperties} object of the camera.
     */
    public PhotonSubsystem(CameraProperties m_camProperties) {
        this.m_camProperties = m_camProperties;
        cameraName = m_camProperties.getCameraName();
        m_camera = new PhotonCamera(cameraName);
        camToRobotTsf = m_camProperties.getCamToRobotTsf();
        m_swerveDrive = RobotContainer.m_swerveSubsystem.getSwerveDrive();
        m_poseEstimator = new PhotonPoseEstimator(
                VisionConstants.aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camToRobotTsf
        );

        m_photonSim = RobotContainer.m_cameraSim;
        m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        m_poseEstimator.setReferencePose(m_swerveDrive.getPose());
    }

    /**
     * Displays how many April Tags the camera can see in the form of an enumeration.
     *
     */
    public enum DetectedTags {
        /** No April Tags are seen by the camera. */
        NONE,
        /** One April Tag is seen by the camera. */
        ONE,
        /** Two April Tags are seen by the camera. */
        TWO,
        /** 3+ April Tags are seen by the camera. */
        MULTIPLE,
    }

    public DetectedTags getDetectionStatus() {
        return detectionStatus;
    }

    public void setDetectionStatus(DetectedTags status) {
        detectionStatus = status;
    }

    public PhotonCamera getCamera() {
        return m_camera;
    }

    public CameraProperties getCameraProperties() {
        return m_camProperties;
    }

    public Transform3d getCamToRobotTsf() {
        return camToRobotTsf;
    }

    public void setCamToRobotTsf(Transform3d newCamToRoboTsf) {
        camToRobotTsf = newCamToRoboTsf;
    }

    public String getCameraName() {
        return cameraName;
    }

    public void update() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var results : m_camera.getAllUnreadResults()) {
            visionEst = m_poseEstimator.update(results);
            setEstimationStdDevs(visionEst, results.getTargets());
            PhotonTrackedTarget result = results.getBestTarget();
            var estimatedPos = results.getMultiTagResult();
            if (result != null) {
                Transform3d aprilTagOffset = result.getBestCameraToTarget().plus(camToRobotTsf.inverse());
                aprilTagTx = aprilTagOffset.getX();
                aprilTagTy = aprilTagOffset.getY();
                aprilTagRot = Math.toDegrees(aprilTagOffset.getRotation().getZ());
                aprilTagID = result.fiducialId;
                aprilTagArea = result.getArea();
                detectedTagsCount = results.getTargets().size();


                switch (detectedTagsCount) {
                    case 1 -> setDetectionStatus(DetectedTags.ONE);
                    case 2 -> setDetectionStatus(DetectedTags.TWO);
                    default -> setDetectionStatus(DetectedTags.MULTIPLE);
                };
            }
            else {
                setDetectionStatus(DetectedTags.NONE);
            }


            if (Robot.isSimulation()) {
                if (m_photonSim != null) {
                    visionEst.ifPresentOrElse(
                            est ->
                                    m_photonSim.getSimDebugField()
                                            .getObject("VisionEstimation")
                                            .setPose(est.estimatedPose.toPose2d()),
                            () -> {
                                m_photonSim.getSimDebugField().getObject("VisionEstimation").setPoses();
                            });
                }
            }

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        Pose2d estPose = est.estimatedPose.toPose2d();
                        m_swerveDrive.addVisionMeasurement(estPose, est.timestampSeconds, estStdDevs);
                    });
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public void setEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = m_poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public double getTagTX() {
        return aprilTagTx;
    }

    public double getTagTY() {
        return aprilTagTy;
    }

    public double getTagRot() {
        return aprilTagRot;
    }

    public int getTagID() {
        return aprilTagID;
    }

    public double getTagArea() {
        return aprilTagArea;
    }

    /**
     * <p>This method updates the telemetry data on SmartDashboard.
     */
    public void updateDashboard() {
        SmartDashboard.putNumber(cameraName + " TAG TX", aprilTagTx);
        SmartDashboard.putNumber(cameraName + " TAG TY", aprilTagTy);
        SmartDashboard.putNumber(cameraName + " TAG ROT", aprilTagRot);
        SmartDashboard.putNumber(cameraName + " TAG ID", aprilTagID);
        SmartDashboard.putNumber(cameraName + " TAG AREA", aprilTagArea);
        SmartDashboard.putNumber(cameraName + " DETECTED TAGS", detectedTagsCount);
        SmartDashboard.putString(cameraName + " Detection Status", getDetectionStatus().toString());

        SmartDashboard.putData(m_camera.);
    }

    @Override
    public void periodic() {
        // Updates the April Tag data (such as its offset).
        update();

        // Updates telemetry data onto SmartDashboard.
        updateDashboard();

    }

}
