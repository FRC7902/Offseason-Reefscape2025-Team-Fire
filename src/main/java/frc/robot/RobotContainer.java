// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.EndEffectorCommands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.FunnelCommands;
import frc.robot.commands.end_effector.IntakeCommand.IntakeMode;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.commands.MoveElevatorArmCommand;
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
    public final static EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();
    public final static ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    public final static ArmSubsystem m_armSubsystem = new ArmSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandPS5Controller m_driverController = new CommandPS5Controller(
            OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
            OperatorConstants.OPERATOR_CONTROLLER_PORT);

    public final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(
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

    private Command coralHandoffCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        // Move elevator to pickup position
                        new ParallelRaceGroup(
                                new MoveElevatorArmCommand(ElevatorPosition.ZERO)
                                        .until(
                                                () -> m_elevatorSubsystem.getElevatorPositionEnum() == ElevatorPosition.ZERO
                                                        && m_funnelIndexerSubsystem.getHasCoral()
                                        ),
                                FunnelCommands.IntakeCoral(m_funnelIndexerSubsystem)
                        ),
                        // Intake coral until funnel no longer detects it (shallow beam break)
                        new ParallelCommandGroup(
                                EndEffectorCommands.IntakeEffector(IntakeMode.CORAL),
                                FunnelCommands.OuttakeCoral(m_funnelIndexerSubsystem)
                        ).until(
                                () -> !m_funnelIndexerSubsystem.getHasCoral()
                        ),
                        // Run end effector intake, funnel intake, and move elevator + arm to level 1 simultaneously
                        new ParallelCommandGroup(
                                EndEffectorCommands.IntakeEffector(IntakeMode.CORAL),
                                FunnelCommands.OuttakeCoral(m_funnelIndexerSubsystem),
                                new MoveElevatorArmCommand(ElevatorPosition.CORAL_L1)
                        )
                ),
                new InstantCommand(),
                // Only run handoff if we don't already have coral and algae
                () -> !m_endEffectorSubsystem.hasCoral()
                        && !m_endEffectorSubsystem.getHasAlgae()
        );
    }

    private ElevatorPosition select() {
        return m_elevatorSubsystem.getElevatorPositionEnum();
    }

    private final Command selectIntakeCommand = new SelectCommand<>(
            Map.ofEntries(
                    Map.entry(ElevatorPosition.ALGAE_LOW, EndEffectorCommands.IntakeEffector(IntakeMode.ALGAE)),
                    Map.entry(ElevatorPosition.ALGAE_HIGH, EndEffectorCommands.IntakeEffector(IntakeMode.ALGAE)),
                    Map.entry(ElevatorPosition.ZERO, coralHandoffCommand()),
                    Map.entry(ElevatorPosition.CORAL_L1, coralHandoffCommand()),
                    Map.entry(ElevatorPosition.CORAL_L2, coralHandoffCommand()),
                    Map.entry(ElevatorPosition.CORAL_L3, coralHandoffCommand()),
                    Map.entry(ElevatorPosition.CORAL_L4, coralHandoffCommand()),
                    Map.entry(ElevatorPosition.BARGE, coralHandoffCommand()),
                    Map.entry(ElevatorPosition.PROCESSOR, coralHandoffCommand())
            ),
            this::select
    );

    private final Command m_selectOuttakeCommand = new SelectCommand<>(Map.ofEntries(
            Map.entry(ElevatorPosition.CORAL_L1, EndEffectorCommands.OuttakeEffector()),
            Map.entry(ElevatorPosition.CORAL_L2, EndEffectorCommands.OuttakeEffector()),
            Map.entry(ElevatorPosition.CORAL_L3, EndEffectorCommands.OuttakeEffector()),
            Map.entry(ElevatorPosition.CORAL_L4, EndEffectorCommands.OuttakeEffector()),
            Map.entry(ElevatorPosition.BARGE, EndEffectorCommands.OuttakeEffector()),
            Map.entry(ElevatorPosition.PROCESSOR, EndEffectorCommands.OuttakeEffector())
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

        m_funnelIndexerSubsystem.setDefaultCommand(
                new ConditionalCommand(
                        new InstantCommand(),
                        FunnelCommands.IntakeCoral(m_funnelIndexerSubsystem),
                        m_endEffectorSubsystem::hasCoral
                )
        );

        // === Intake/Outtake controls ===
        m_driverController.R2().whileTrue(EndEffectorCommands.OuttakeEffector());
        m_driverController.L2().whileTrue(
                selectIntakeCommand
                        .until(() -> !m_driverController.L2()
                                .getAsBoolean()
                        )
        );
        // ===============================

        m_driverController.options().onTrue(new InstantCommand(m_swerveSubsystem::zeroGyro));

        // === Elevator Setpoints ===
        m_operatorController.triangle().onTrue(
                new MoveElevatorArmCommand(ElevatorPosition.CORAL_L4)
        );
        m_operatorController.circle().onTrue(
                new MoveElevatorArmCommand(ElevatorPosition.CORAL_L3)
        );
        m_operatorController.square().onTrue(
                new MoveElevatorArmCommand(ElevatorPosition.CORAL_L2)
        );
        m_operatorController.cross().onTrue(
                new MoveElevatorArmCommand(ElevatorPosition.CORAL_L1)
        );

        m_operatorController.povDown().onTrue(new MoveElevatorArmCommand(ElevatorPosition.ZERO));
        m_operatorController.povUp().onTrue(new MoveElevatorArmCommand(ElevatorPosition.PROCESSOR));
        m_operatorController.povLeft().onTrue(new MoveElevatorArmCommand(ElevatorPosition.ALGAE_HIGH));
        m_operatorController.povRight().onTrue(new MoveElevatorArmCommand(ElevatorPosition.ALGAE_LOW));
        // ==========================

        m_operatorController.options().whileTrue(m_swerveSubsystem.centerModulesCommand());
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