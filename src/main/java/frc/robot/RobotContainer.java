// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.commands.FunnelCommands;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.PhotonSim;
import frc.robot.subsystems.vision.PhotonSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final FunnelSubsystem m_funnelIndexerSubsystem = new FunnelSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private static final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT);

    public static final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(
            m_driverController,
            new File(Filesystem.getDeployDirectory(), "swerve"));

    public static PhotonSim m_cameraSim;
    public final PhotonSubsystem m_middleCamera = new PhotonSubsystem(PhotonConstants.middleCamProp);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (Robot.isSimulation()) {
                // m_cameraSim = new PhotonSim(m_swerveSubsystem, m_leftCamera, m_middleCamera, m_rightCamera);
                //m_cameraSim = new PhotonSim(m_swerveSubsystem, m_middleCamera);
          
                  driveRobotOriented.driveToPose(m_swerveSubsystem::getNearestWaypoint,
                          new ProfiledPIDController(5,
                                  0,
                                  0,
                                  new TrapezoidProfile.Constraints(5, 2)),
                          new ProfiledPIDController(5,
                                  0,
                                  0,
                                  new TrapezoidProfile.Constraints(Units.degreesToRadians(360),
                                          Units.degreesToRadians(180))));
          
                  m_driverController.start().whileTrue(Commands.runEnd(
                          () -> driveRobotOriented.driveToPoseEnabled(true),
                          () -> driveRobotOriented.driveToPoseEnabled(false)
                  ));
              }        
        
        // Configure the trigger bindings
        configureBindings();
    }

    public SwerveInputStream driveAngularVelocity = SwerveInputStream
            .of(m_swerveSubsystem.getSwerveDrive(), () -> m_driverController.getLeftY() * -1,
                    () -> m_driverController.getLeftX() * -1)
            .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
            .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
            .allianceRelativeControl(true);

    public SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
            .withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightY)
            .headingWhile(true);

    Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveRobotOriented);

    SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                    () -> -m_driverController.getLeftY(),
                    () -> -m_driverController.getLeftX())
            .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                    2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
            .withControllerHeadingAxis(() -> Math.sin(
                            m_driverController.getRawAxis(
                                    2) *
                                    Math.PI)
                            *
                            (Math.PI *
                                    2),
                    () -> Math.cos(
                            m_driverController.getRawAxis(
                                    2) *
                                    Math.PI)
                            *
                            (Math.PI *
                                    2))
            .headingWhile(true)
            .translationHeadingOffset(true)
            .translationHeadingOffset(Rotation2d.fromDegrees(
                    0));
    Command driveFieldOrientedDirectAngleSim = m_swerveSubsystem.driveFieldOriented(driveDirectAngleSim);

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        // Swerve
        m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

        // Auto-Algin
        driveAngularVelocity.driveToPose(m_swerveSubsystem::getNearestWaypoint,
                new ProfiledPIDController(5,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(5, 2)),
                new ProfiledPIDController(5,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(Units.degreesToRadians(360),
                                Units.degreesToRadians(180))));

        m_driverController.start().whileTrue(Commands.runEnd(
                () -> driveAngularVelocity.driveToPoseEnabled(true),
                () -> driveAngularVelocity.driveToPoseEnabled(false)
        ));

        // FunnelSubsystem
        m_funnelIndexerSubsystem.setDefaultCommand(FunnelCommands.IntakeCoral(m_funnelIndexerSubsystem));
        m_driverController.rightBumper().whileTrue(FunnelCommands.OuttakeCoral(m_funnelIndexerSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}