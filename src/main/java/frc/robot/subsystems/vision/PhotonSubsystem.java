package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase {

    private CameraProperties m_camProperties;
    private PhotonCamera m_camera;
    private Transform3d camToRobotTsf;
    private DetectedTags detectionStatus = DetectedTags.NONE;
    private String cameraName;

    private double aprilTagRot = 0;
    private double aprilTagTx = 0;
    private double aprilTagTy = 0;
    private double aprilTagArea = 0;
    private int aprilTagID = 0;
    private int detectedTagsCount = 0;


    /**
     * Creates the PhotonVision camera object. This is where all the camera data is processed. 
     * If using a real camera, ensure the camera name matches the name set in PhotonVision. Otherwise, it will not detect the camera.
     * @param CameraProperties The {@code CameraProperties} object of the camera.
    */ 
    public PhotonSubsystem(CameraProperties m_camProperties) {
        this.m_camProperties = m_camProperties;
        cameraName = m_camProperties.getCameraName();
        m_camera = new PhotonCamera(cameraName);
        camToRobotTsf = m_camProperties.getCamToRobotTsf();
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
        for (var results : m_camera.getAllUnreadResults()) {
            PhotonTrackedTarget result = results.getBestTarget();
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
    }

    @Override
    public void periodic() {
        // Updates the April Tag data (such as its offset).
        update();

        // Updates telemetry data onto SmartDashboard.
        updateDashboard();
    }

}
