// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.EndEffectorCommands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.FunnelCommands;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.commands.ElevatorArmCommands.MoveElevatorArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

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
    private final EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();
    public final static ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    public final static ArmSubsystem m_armSubsystem = new ArmSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT);

    public final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(
            m_driverController,
            new File(Filesystem.getDeployDirectory(), "swerve"));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
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
        // FunnelSubsystem
        m_funnelIndexerSubsystem.setDefaultCommand(FunnelCommands.IntakeCoral(m_funnelIndexerSubsystem));
//        m_driverController.rightBumper().whileTrue(FunnelCommands.OuttakeCoral(m_funnelIndexerSubsystem));

        // EndEffectorSubsystem
//        m_driverController.leftTrigger().whileTrue(EndEffectorCommands.IntakeEffector(m_endEffectorSubsystem));
//        m_driverController.rightTrigger().whileTrue(EndEffectorCommands.OuttakeEffector(m_endEffectorSubsystem));

        // Intake coral
//        m_driverController.leftTrigger().whileTrue(
//                new MoveElevatorArmCommand(ElevatorPosition.ZERO)
//                        .andThen(EndEffectorCommands.IntakeEffector(m_endEffectorSubsystem))
//        );

        m_driverController.leftTrigger().whileTrue(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                // Move elevator to pickup position
                                new MoveElevatorArmCommand(ElevatorPosition.ZERO),
                                // Intake coral until funnel no longer detects it (shallow beam break)
                                new ParallelCommandGroup(
                                        EndEffectorCommands.IntakeEffector(m_endEffectorSubsystem),
                                        FunnelCommands.OuttakeCoral(m_funnelIndexerSubsystem)
                                ).until(
                                        () -> !m_funnelIndexerSubsystem.getHasCoral()
                                ),
                                // Run end effector intake, funnel intake, and move elevator + arm to level 1 simultaneously
                                new ParallelCommandGroup(
                                        EndEffectorCommands.IntakeEffector(m_endEffectorSubsystem),
                                        FunnelCommands.OuttakeCoral(m_funnelIndexerSubsystem),
                                        new MoveElevatorArmCommand(ElevatorPosition.CORAL_L1)
                                )
                        ),
                        new InstantCommand(),
                        m_funnelIndexerSubsystem::getHasCoral
                )
        );

        // Elevator Setpoints
        m_driverController.y().onTrue(new MoveElevatorArmCommand(ElevatorPosition.CORAL_L4));
        m_driverController.b().onTrue(new MoveElevatorArmCommand(ElevatorPosition.CORAL_L3));
        m_driverController.x().onTrue(new MoveElevatorArmCommand(ElevatorPosition.CORAL_L2));
        m_driverController.a().onTrue(new MoveElevatorArmCommand(ElevatorPosition.CORAL_L1));

//        m_driverController.rightBumper().onTrue(new MoveElevatorArmCommand(ElevatorPosition.ALGAE_HIGH));
//        m_driverController.leftBumper().onTrue(new MoveElevatorArmCommand(ElevatorPosition.ALGAE_LOW));

        m_driverController.povDown().onTrue(new MoveElevatorArmCommand(ElevatorPosition.ZERO));
//        m_driverController.leftStick().onTrue(new MoveElevatorArmCommand(ElevatorPosition.BARGE));
//
//        m_driverController.rightStick().onTrue(new MoveElevatorArmCommand(ElevatorPosition.PROCESSOR));

        m_swerveSubsystem.setDefaultCommand(
                Robot.isSimulation() ? driveFieldOrientedAngularVelocity : driveRobotOrientedAngularVelocity);
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