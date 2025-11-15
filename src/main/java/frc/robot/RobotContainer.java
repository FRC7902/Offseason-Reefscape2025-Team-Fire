// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
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
    public final static ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    public final static ArmSubsystem m_armSubsystem = new ArmSubsystem();

    public final static SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(
            new File(Filesystem.getDeployDirectory(), "swerve"));

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final static CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final static CommandXboxController m_operatorController = new CommandXboxController(
            OperatorConstants.OPERATOR_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        configurePathPlanner();

        autoChooser = AutoBuilder.buildAutoChooser("DEFAULT");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public static SwerveInputStream driveAngularVelocity = SwerveInputStream
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

    private ElevatorPosition select() {
        return m_elevatorSubsystem.getElevatorArmPositionEnum();
    }

    private final Command selectIntakeCommand = new SelectCommand<>(
            Map.ofEntries(
//                    Map.entry(ElevatorPosition.ALGAE_LOW, EndEffectorCommands.IntakeEffector(IntakeMode.ALGAE)),
            ),
            this::select);

    private final Command m_selectOuttakeCommand = new SelectCommand<>(Map.ofEntries(
//            Map.entry(ElevatorPosition.CORAL_L1, EndEffectorCommands.OuttakeEffector()
    ),
            this::select);

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

        m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

        // === Intake/Outtake controls ===
        m_driverController.rightTrigger().whileTrue(
                m_selectOuttakeCommand
                        .until(() -> !m_driverController.rightTrigger()
                                .getAsBoolean())
        );
        m_driverController.leftTrigger().whileTrue(
                selectIntakeCommand
                        .until(() -> !m_driverController.leftTrigger()
                                .getAsBoolean()));
        // ===============================

        // Strafe controls
        m_driverController.povLeft().whileTrue(SwereCommands.StrafeLeft());
        m_driverController.povRight().whileTrue(SwereCommands.StrafeRight());

        m_driverController.start().onTrue(new InstantCommand(m_swerveSubsystem::zeroGyro));
        m_driverController.back().onTrue(new InstantCommand(m_swerveSubsystem::toggleFastDriveRampRateMode));

        // === Elevator Setpoints ===
        m_operatorController.y().onTrue(
                new MoveElevatorArmCommand(ElevatorPosition.CORAL_L4)
        );
        m_operatorController.b().onTrue(
                new MoveElevatorArmCommand(ElevatorPosition.CORAL_L3)
        );
        m_operatorController.x().onTrue(
                new MoveElevatorArmCommand(ElevatorPosition.CORAL_L2)
        );
        m_operatorController.a().onTrue(
                new MoveElevatorArmCommand(ElevatorPosition.CORAL_L1)
        );
        m_operatorController.back().onTrue(
                new MoveElevatorArmCommand(ElevatorPosition.MIDDLE)
        );
        m_operatorController.povUp().onTrue(new MoveElevatorArmCommand(ElevatorPosition.BARGE));
        m_operatorController.povDown().onTrue(new MoveElevatorArmCommand(ElevatorPosition.PROCESSOR));
        m_operatorController.povLeft().onTrue(new MoveElevatorArmCommand(ElevatorPosition.ALGAE_HIGH));
        m_operatorController.povRight().onTrue(new MoveElevatorArmCommand(ElevatorPosition.ALGAE_LOW));
        // ==========================

        m_operatorController.start().whileTrue(m_swerveSubsystem.centerModulesCommand());
    }

    private void configurePathPlanner() {
        new EventTrigger("ELV_ZERO").onTrue(new MoveElevatorArmCommand(
                ElevatorPosition.ZERO));
        new EventTrigger("ELV_REST").onTrue(new MoveElevatorArmCommand(
                ElevatorPosition.REST));
        new EventTrigger("ELV_L1").onTrue(new MoveElevatorArmCommand(
                ElevatorPosition.CORAL_L1));
        new EventTrigger("ELV_L2").onTrue(new MoveElevatorArmCommand(
                ElevatorPosition.CORAL_L2));
        new EventTrigger("ELV_L3").onTrue(new MoveElevatorArmCommand(
                ElevatorPosition.CORAL_L3));
        new EventTrigger("ELV_L4").onTrue(new MoveElevatorArmCommand(
                ElevatorPosition.CORAL_L4));
        new EventTrigger("ELV_ALG_LOW").onTrue(new MoveElevatorArmCommand(
                ElevatorPosition.ALGAE_LOW));
        new EventTrigger("ELV_ALG_HIGH").onTrue(new MoveElevatorArmCommand(
                ElevatorPosition.ALGAE_HIGH));
        new EventTrigger("ELV_ALG_BARGE").onTrue(new MoveElevatorArmCommand(
                ElevatorPosition.BARGE));
        new EventTrigger("ELV_ALG_PROCESSOR").onTrue(new MoveElevatorArmCommand(
                ElevatorPosition.PROCESSOR));

        new EventTrigger("AUTO_ALIGN_CENTER").onTrue(AutoAlignCommands.AutoAlignCenter());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

    }
}
