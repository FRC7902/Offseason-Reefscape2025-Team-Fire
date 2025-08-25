package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class PhotonSim extends SubsystemBase {

    private final SwerveSubsystem m_swerveSubsystem;
    private final VisionSystemSim m_visionSim;
    private final PhotonSubsystem[] cameras;
    private final ArrayList<PhotonCameraSim> m_cameraSims;

    /**
     * Creates the PhotonSim object. This is where all the vision simulation happens. All of the camera objects should be placed in its arguments. 
     * @param SwerveSubsystem The {@code SwerveSubsystem} object of the robot.
     * @param PhotonSubsystem The Camera objects created (accepts an unlimited amount of arguments).
    */     
    public PhotonSim(SwerveSubsystem m_swerveSubsystem, PhotonSubsystem... cameras) {

        // Instantiate DriveSubsystem object, used for obtaining robot pose.
        this.m_swerveSubsystem = m_swerveSubsystem;
        this.cameras = cameras;

        // Creates the Vision Simulation Object
        m_visionSim = new VisionSystemSim("main");

        // Adds the position of the April Tags onto the vision simulator. Used for viewing the april tags from the simulated camera view.
        m_visionSim.addAprilTags(VisionConstants.aprilTagFieldLayout);

        // Creates a simulation camera configurator object, which is used to set some settings for the camera.
        // Ideally should match the specifications of the cameras that are on the robot.
        SimCameraProperties cameraSettings = new SimCameraProperties();

        cameraSettings.setAvgLatencyMs(35);
        cameraSettings.setLatencyStdDevMs(5);

        m_cameraSims = new ArrayList<PhotonCameraSim>();

        for (PhotonSubsystem camera : cameras) {
            Transform3d camToRobotTsf = camera.getCamToRobotTsf();
            PhotonCamera m_camera = camera.getCamera();
            String cameraName = camera.getCameraName();
            CameraProperties m_camProperties = camera.getCameraProperties();

            int resWidth = m_camProperties.getResWidth();
            int resHeight = m_camProperties.getResHeight();
            Rotation2d fov = m_camProperties.getFov();
            int fps = m_camProperties.getFps();
            double avgErrorPx = m_camProperties.getAvgErrorPx();
            double errorStdDevPx = m_camProperties.getErrorStdDevPx();

            // Sets the resolution of the camera.
            // For example, a 640 x 480 camera with a 100 degree diagonal FOV.
            cameraSettings.setCalibration(resWidth, resHeight, fov);
            // Sets the approximate detection noise with average and standard deviation error in pixels.
            // This can be used to determine how much calibration error can affect reliability (can use the calibration error from the actual camera calibration results).
            cameraSettings.setCalibError(avgErrorPx, errorStdDevPx);
            // Set the camera image capture framerate (Note: this is limited by robot loop rate).
            cameraSettings.setFPS(fps);
            // The average and standard deviation in milliseconds of image data latency.

            // Creates the simulated PhotonVision camera object.
            // Uses the PhotonVision camera object and adds in the simulated camera configuration to create a simuluated camera view. 
            PhotonCameraSim m_cameraSim = new PhotonCameraSim(m_camera, cameraSettings);
            m_cameraSims.add(m_cameraSim);

            // Sets the position of the camera relative to the center of the robot
            // In this example, our camera is mounted 0.1 meters forward (x) and 0.5 meters up (y) from the robot pose
            // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
            Translation3d robotToCameraTrl = camToRobotTsf.getTranslation();

            // Sets the pitch and the rotation of the camera relative to the robot.
            // In this example, the camera is pitched 15 degrees up and rotated 0 degrees.
            Rotation3d robotToCameraRot = camToRobotTsf.getRotation();

            // Combines the translational and rotational data of the camera into one Transform3d object.
            Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

            // Adds the simulated camera (with its configuration) alongside the camera's position on the robot
            // This creates the camera pose on the robot, which, depending on how offsetted it is from the robot, can be seen in odometry via Glass (Robot Simulation)
            // Click on the NetworkTables tab -> SmartDashboard -> VisionSystemSim-main -> Sim Field
            m_visionSim.addCamera(m_cameraSim, robotToCamera);

            // Lets you see the simulated camera's view, which also shows which April Tags are seen in its view
            // To see this simulated view, on your broswer, go to: localhost:1182 (increments of 2 for each new camera, e.g: localhost:1182, localhost:1184, localhost:1186, etc...)
            // NOTE: You must be currently simulating to be able to see this! It is also an expensive process, so your computer may lag when this is enabled.
            m_cameraSim.enableDrawWireframe(true);

            Rotation3d camRot = camToRobotTsf.getRotation();

            SmartDashboard.putNumber(cameraName + " X TRANSFORM", camToRobotTsf.getX());
            SmartDashboard.putNumber(cameraName + " Y TRANSFORM", camToRobotTsf.getY());
            SmartDashboard.putNumber(cameraName + " Z TRANSFORM", camToRobotTsf.getZ());
            SmartDashboard.putNumber(cameraName + " ROTATION OFFSET", Math.toDegrees(camRot.getZ()));
            SmartDashboard.putNumber(cameraName + " PITCH OFFSET", Math.toDegrees(camRot.getY()));

        }

    }


    @Override
    public void simulationPeriodic() {
        // Gets the robot pose
        Pose2d robotPose = m_swerveSubsystem.getPose();

        // A Field2d object representing the robot and the April Tags on the field
        var debugField = m_visionSim.getDebugField();

        // Updates the robot pose in the Field2d Object
        debugField.getObject("EstimatedRobot").setPose(robotPose);

        // Updates the robot pose in the vision sim so that the camera's position is moving alongside the robot.
        m_visionSim.update(robotPose);


        for (int i = 0; i != m_cameraSims.size(); i++) {
            PhotonCameraSim m_cameraSim = m_cameraSims.get(i);
            PhotonSubsystem camera = cameras[i];

            String cameraName = camera.getCameraName();
            Transform3d camToRobotTsf = camera.getCamToRobotTsf();
            Rotation3d camRot = camToRobotTsf.getRotation();

            double x = SmartDashboard.getNumber(cameraName + " X TRANSFORM", camToRobotTsf.getX());
            double y = SmartDashboard.getNumber(cameraName + " Y TRANSFORM", camToRobotTsf.getY());
            double z = SmartDashboard.getNumber(cameraName + " Z TRANSFORM", camToRobotTsf.getZ());

            double yaw = SmartDashboard.getNumber(cameraName + " YAW OFFSET", Math.toDegrees(camRot.getZ()));
            double pitch = SmartDashboard.getNumber(cameraName + " PITCH OFFSET", Math.toDegrees(camRot.getY()));

            Transform3d newCamToRobotTsf = new Transform3d(x, y, z, (new Rotation3d(0, Math.toRadians(pitch), Math.toRadians(yaw))));

            camera.setCamToRobotTsf(newCamToRobotTsf);

            m_visionSim.adjustCamera(m_cameraSim, camera.getCamToRobotTsf());
        }
    }
}
