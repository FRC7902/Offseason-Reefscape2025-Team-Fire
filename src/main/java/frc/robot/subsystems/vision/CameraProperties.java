package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraProperties {

    private final Transform3d camToRobotTsf;
    private final String camName;
    private final int resWidth;
    private final int resHeight;
    private final Rotation2d fov;
    private final int fps;
    private final double avgErrorPx;
    private final double errorStdDevPx;

    /**
     * Creates a new {@code CameraProperty} object, which represents all of the
     * properties of the actual camera to be used in simulation.
     *
     * @param camName The name of the camera (e.g: {@code leftCamera}, {@code bargeCamera}, etc).
     * @param camToRobotTsf The offset of the camera from the center of the robot.
     * @param resWidth The resolution width of the camera.
     * @param resHeight The resolution height of the camera.
     * @param fov The FOV (Field of View) of the camera (in degrees).
     * @param fps The maximum FPS the camera can support
     * @param avgErrorPx The average error in pixels (this should be the same from the calibration results if a real camera was used).
     * @param errorStdDevPx The uncertainty in measurements or estimation taken by vision.
     */
    public CameraProperties(String camName, Transform3d camToRobotTsf, int resWidth, int resHeight, Rotation2d fov, int fps, double avgErrorPx, double errorStdDevPx) {
        this.camName = camName;
        this.camToRobotTsf = camToRobotTsf;
        this.resWidth = resWidth;
        this.resHeight = resHeight;
        this.fov = fov;
        this.fps = fps;
        this.avgErrorPx = avgErrorPx;
        this.errorStdDevPx = errorStdDevPx;
    }

    public Transform3d getCamToRobotTsf() {
        return camToRobotTsf;
    }

    public String getCameraName() {
        return camName;
    }

    public int getResWidth() {
        return resWidth;
    }

    public int getResHeight() {
        return resHeight;
    }

    public Rotation2d getFov() {
        return fov;
    }

    public int getFps() {
        return fps;
    }

    public double getAvgErrorPx() {
        return avgErrorPx;
    }

    public double getErrorStdDevPx() {
        return errorStdDevPx;
    }

}
