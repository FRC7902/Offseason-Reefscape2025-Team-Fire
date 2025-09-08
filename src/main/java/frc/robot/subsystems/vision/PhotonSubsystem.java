package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.prefs.InvalidPreferencesFormatException;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class PhotonSubsystem extends SubsystemBase {
    private ElevatorPosition height;
    public static HashMap<Integer,Boolean> reefIDHeights = new HashMap<>();
    // High = true
    // Low = false
    public enum IDReefHeights{
        // blue alliance
        ID_17,
        ID_18,
        ID_19,
        ID_20,
        ID_21,
        ID_22,
        // red alliance
        ID_6,
        ID_7,
        ID_8,
        ID_9,
        ID_10,
        ID_11
    }
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
    public ElevatorPosition getHeight() {
        return height;
    }
    // Checks if apriltag is on high or algae reef height
    public ElevatorPosition setIDHeights(int id) {
        List<IDReefHeights> highAlgae = Arrays.asList(IDReefHeights.ID_7, IDReefHeights.ID_9,IDReefHeights.ID_11, IDReefHeights.ID_18, IDReefHeights.ID_20,IDReefHeights.ID_22);
        IDReefHeights enumID = switch (id) {
            case 6 -> IDReefHeights.ID_6;
            case 7 -> IDReefHeights.ID_7;
            case 8 -> IDReefHeights.ID_8;
            case 9 -> IDReefHeights.ID_9;
            case 10 -> IDReefHeights.ID_10;
            case 11 -> IDReefHeights.ID_11;

            case 17 -> IDReefHeights.ID_17;
            case 18 -> IDReefHeights.ID_18;
            case 19 -> IDReefHeights.ID_19;
            case 20 -> IDReefHeights.ID_20;
            case 21 -> IDReefHeights.ID_21;
            default -> null;

        };
        if (highAlgae.contains(enumID)){
            height = ElevatorPosition.ALGAE_HIGH;
        }
        else {
            height = ElevatorPosition.ALGAE_LOW;
        }
        return height;
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
