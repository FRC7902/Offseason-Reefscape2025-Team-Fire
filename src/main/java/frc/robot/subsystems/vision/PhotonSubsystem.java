package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase {

    private final CameraProperties m_camProperties;
    private final PhotonCamera m_camera;
    private Transform3d m_camToRobotTsf;
    private DetectedTags m_detectionStatus = DetectedTags.NONE;
    private final String m_cameraName;

    private double m_aprilTagRot = 0;
    private double m_aprilTagTx = 0;
    private double m_aprilTagTy = 0;
    private double m_aprilTagArea = 0;
    private int m_aprilTagID = 0;
    private int m_detectedTagsCount = 0;


    /**
     * Creates the PhotonVision camera object. This is where all the camera data is processed.
     * If using a real camera, ensure the camera name matches the name set in PhotonVision. Otherwise, it will not detect the camera.
     *
     * @param m_camProperties The {@code CameraProperties} object of the camera.
     */
    public PhotonSubsystem(CameraProperties m_camProperties) {
        this.m_camProperties = m_camProperties;
        m_cameraName = m_camProperties.getCameraName();
        m_camera = new PhotonCamera(m_cameraName);
        m_camToRobotTsf = m_camProperties.getCamToRobotTsf();
    }

    /**
     * Displays how many April Tags the camera can see in the form of an enumeration.
     *
     */
    public enum DetectedTags {
        /**
         * No April Tags are seen by the camera.
         */
        NONE,
        /**
         * One April Tag is seen by the camera.
         */
        ONE,
        /**
         * Two April Tags are seen by the camera.
         */
        TWO,
        /**
         * 3+ April Tags are seen by the camera.
         */
        MULTIPLE,
    }

    public DetectedTags getM_detectionStatus() {
        return m_detectionStatus;
    }

    public void setM_detectionStatus(DetectedTags status) {
        m_detectionStatus = status;
    }

    public PhotonCamera getCamera() {
        return m_camera;
    }

    public CameraProperties getCameraProperties() {
        return m_camProperties;
    }

    public Transform3d getM_camToRobotTsf() {
        return m_camToRobotTsf;
    }

    public void setM_camToRobotTsf(Transform3d newCamToRoboTsf) {
        m_camToRobotTsf = newCamToRoboTsf;
    }

    public String getM_cameraName() {
        return m_cameraName;
    }

    public void update() {
        for (var results : m_camera.getAllUnreadResults()) {
            PhotonTrackedTarget result = results.getBestTarget();
            if (result != null) {
                Transform3d aprilTagOffset = result.getBestCameraToTarget().plus(m_camToRobotTsf.inverse());
                m_aprilTagTx = aprilTagOffset.getX();
                m_aprilTagTy = aprilTagOffset.getY();
                m_aprilTagRot = Math.toDegrees(aprilTagOffset.getRotation().getZ());
                m_aprilTagID = result.fiducialId;
                m_aprilTagArea = result.getArea();
                m_detectedTagsCount = results.getTargets().size();

                switch (m_detectedTagsCount) {
                    case 1 -> setM_detectionStatus(DetectedTags.ONE);
                    case 2 -> setM_detectionStatus(DetectedTags.TWO);
                    default -> setM_detectionStatus(DetectedTags.MULTIPLE);
                }
            } else {
                setM_detectionStatus(DetectedTags.NONE);
            }
        }
    }

    public double getTagTX() {
        return m_aprilTagTx;
    }

    public double getTagTY() {
        return m_aprilTagTy;
    }

    public double getTagRot() {
        return m_aprilTagRot;
    }

    public int getTagID() {
        return m_aprilTagID;
    }

    public double getTagArea() {
        return m_aprilTagArea;
    }

    /**
     * <p>This method updates the telemetry data on SmartDashboard.
     */
    public void updateDashboard() {
        SmartDashboard.putNumber(m_cameraName + " TAG TX", m_aprilTagTx);
        SmartDashboard.putNumber(m_cameraName + " TAG TY", m_aprilTagTy);
        SmartDashboard.putNumber(m_cameraName + " TAG ROT", m_aprilTagRot);
        SmartDashboard.putNumber(m_cameraName + " TAG ID", m_aprilTagID);
        SmartDashboard.putNumber(m_cameraName + " TAG AREA", m_aprilTagArea);
        SmartDashboard.putNumber(m_cameraName + " DETECTED TAGS", m_detectedTagsCount);
        SmartDashboard.putString(m_cameraName + " Detection Status", getM_detectionStatus().toString());
    }

    @Override
    public void periodic() {
        // Updates the April Tag data (such as its offset).
        update();

        // Updates telemetry data onto SmartDashboard.
        updateDashboard();
    }

}
