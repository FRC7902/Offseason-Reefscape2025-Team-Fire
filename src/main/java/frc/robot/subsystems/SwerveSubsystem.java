// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;
  public SwerveSubsystem(File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED,
                    new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)),
                            Rotation2d.fromDegrees(0)));
    }
    catch (Exception e){
      throw new RuntimeException(e);
    }
    if (Robot.isSimulation()) {
      swerveDrive.setHeadingCorrection(false);
      swerveDrive.setCosineCompensator(false);
    } else {
        swerveDrive.setCosineCompensator(true);
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be
        // used while controlling the robot via angle.
    }
    swerveDrive.setAngularVelocityCompensation(true, false, 0.1);
    // Start with 0.1 for angular velocity coeffecient, adjust as needed
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    // Synchronizes absolute encoders and motor encoders when stationary
    swerveDrive.setChassisDiscretization(false, true, 0.03);
    swerveDrive.swerveController.addSlewRateLimiters(null, null, null);
    swerveDrive.swerveController.setMaximumChassisAngularVelocity(20);
    
    // Insert setuppathplanner method

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro angle rotation (rad)",
      swerveDrive.getGyro().getRotation3d().getAngle());
    // Writes gyro angle rotation in radians            
    SmartDashboard.putString("Robo Pose2D", swerveDrive.getPose().toString());
    // Writes robot pose
    swerveDrive.updateOdometry();
    // Manually update odometery (only during vision test? Might remove)
  }
  // characterizes robot drive motors using sysId
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
          SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true),
          3.0, 5.0, 3.0);
  }
  // characterizes robot angle motors using sysId
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive), 3.0, 5.0,
      3.0);
  }
  // centers swerve drive system modules
  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
  }
  // drives the swerve drive to a particular distance at a given speed
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return run(() -> drive(new ChassisSpeeds(
      RobotContainer.m_driverController.getLeftY() * swerveDrive.getMaximumChassisVelocity(),
      speedInMetersPerSecond, 0)))
      .until(() -> swerveDrive.getPose().getTranslation()
      .getDistance(new Translation2d(0, 0)) > distanceInMeters);
  }
  // replaces the swerve feedforward with the simplemotorfeedforward object
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }
  // returns max angular velocity of chassis
  public double getMaximumChassisAngularVelocity(){
    return swerveDrive.getMaximumChassisAngularVelocity();
  }
  // Drives robot using given translations and given angular velocity
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> {
            // Make the robot move
            swerveDrive.drive(
                SwerveMath.scaleTranslation(new Translation2d(
                translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                0.8),
                Math.pow(angularRotationX.getAsDouble(), 3)
                 * swerveDrive.getMaximumChassisAngularVelocity(),
                true, false);
        });
  }
  // Drives robot using given translations and angular setpoint
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier headingX, DoubleSupplier headingY) {
        // Normally you would want heading
        // correction for this kind of control.
        return run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(
                    new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                    scaledInputs.getY(), headingX.getAsDouble(), headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
  }
  // primary method to control the drive base
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false); 
        // Open loop is disabled
        // since it shouldn't be
        // used most of the time.
  }
  // drive according to chassis oriented field velocity
  public void driveFieldOriented(ChassisSpeeds velocity){
    swerveDrive.driveFieldOriented(velocity);
  }
  /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }
  // drive according to chassis oriented robot velocity
  public void drive(ChassisSpeeds velocity){
    swerveDrive.drive(velocity);
  }
  // resets odometry
  public void resetOdometry(Pose2d IntialHolonomicPose){
    swerveDrive.resetOdometry(IntialHolonomicPose);
  }
  // get swerve kinematics object
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }
  // sets Chassis Speeds using closed-loop control
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }
  // posts trajectory to field
  public void postTrajectory(Trajectory trajectory){
    swerveDrive.postTrajectory(trajectory);
  }
  // zeroes the gyro
  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }
  // checks if red alliance
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }
  // Zero gyro and turn 180 degrees if red alliance, else just
  // zero gyro
  public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            zeroGyro();
            // Set the pose 180 degrees
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            zeroGyro();
        }
    }
  // Set drive motors to brake/coast mode
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }
  // Gets yaw of robot as reported by swerve drivebase
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }
  // Get target speeds from speed in x-y plane and angles
  // Takes controller output from 2 joysticks
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double xHeading, double yHeading) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput,yInput));
    return swerveDrive.swerveController.getTargetSpeeds
    (scaledInputs.getX(), scaledInputs.getY(), xHeading, yHeading, getHeading().getRadians(),SwerveConstants.MAX_SPEED);
  }
  // Takes controller output from 1 joystick and 1 angle
  public ChassisSpeeds getTargetSpeeds(double xInput,double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput,yInput));
    return swerveDrive.swerveController.getTargetSpeeds
    (scaledInputs.getX(), scaledInputs.getY(), angle.getRadians(), getHeading().getRadians(), SwerveConstants.MAX_SPEED);
  }
  //Gets Tangential velocity (x and y) as well as angular velocity (omega)
  //Field Relative
  public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
  }
  // Gets Tangential velocity (x and y) as well as angular velocity (omega)
  // Robot Relative
  public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
  }
  // Gets swervecontroller
  public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
  }
  // Gets swervedrive config
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
  }
  // Turns wheels to face each other, locking the position
  public void lock() {
        swerveDrive.lockPose();
  }
  // Gets pitch angle from imu
  public Rotation2d getPitch(){
    return swerveDrive.getPitch();
  }
  public Pose2d getPose(){
    return swerveDrive.getPose();
  }
  // Get SwerveDrive object
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
  // command to get to angle setpoint
  public Command snapToAngle(double angleDegrees, double toleranceDegrees) {
        SwerveController controller = swerveDrive.getSwerveController();

        return run(() -> {
            swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
                    controller.headingCalculate(
                            swerveDrive.getOdometryHeading().unaryMinus().getRadians(),
                            new Rotation2d(Math.toRadians(angleDegrees)).getRadians()),
                    swerveDrive.getPose().getRotation()));
            SmartDashboard.putNumber("Odom Heading (rad)",
                    swerveDrive.getOdometryHeading().unaryMinus().getRadians());
            SmartDashboard.putNumber("Target Heading (rad)", Math.toRadians(angleDegrees));
            SmartDashboard.putNumber("Error (rad)",
                    Math.abs(new Rotation2d(Math.toRadians(angleDegrees))
                            .minus(swerveDrive.getOdometryHeading().unaryMinus()).getRadians()));
        }).until(() -> (Math.abs(new Rotation2d(Math.toRadians(angleDegrees))
                .minus(swerveDrive.getOdometryHeading().unaryMinus())
                .getDegrees()) < toleranceDegrees)
                && swerveDrive.getRobotVelocity().omegaRadiansPerSecond < 0.1);
    }
  // Doesn't correct strafe, just makes the robot spin vigorously
  public void strafe(double strafePower, double rotationalPower, double speedMultiplier) {
        swerveDrive.drive(new Translation2d(
                strafePower * Math.abs(speedMultiplier) * swerveDrive.getMaximumChassisVelocity(), 0),
                rotationalPower, true, false);

  }
  public void strafe(double strafePower, double speedMultiplier) {
        swerveDrive.drive(new Translation2d(0,
                strafePower * Math.abs(speedMultiplier) * swerveDrive.getMaximumChassisVelocity()),
                0, false, false);

  }

}
