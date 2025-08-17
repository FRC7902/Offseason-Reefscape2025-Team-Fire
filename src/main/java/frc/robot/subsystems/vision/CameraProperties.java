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
