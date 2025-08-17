// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveCommands.StrafeLeftCommand;
import frc.robot.commands.SwerveCommands.StrafeRightCommand;

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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  public static final CommandXboxController m_operatorController = new CommandXboxController(
        OperatorConstants.kOperatorControllerPort);    
  public static final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  /**
  * Converts driver input into a field-relative ChassisSpeeds that is controlled
  * by angular
  * velocity.
  */
  public SwerveInputStream driveAngularVelocity = SwerveInputStream
      .of(m_swerveSubsystem.getSwerveDrive(), () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
      .allianceRelativeControl(true);

/**
 Clone's the angular velocity input stream and converts it to a fieldRelative
* input stream.
*/
SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightY)
      .headingWhile(true);

/**
* Clone's the angular velocity input stream and converts it to a robotRelative
* input stream.
*/
public SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream
    .of(m_swerveSubsystem.getSwerveDrive(), () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX())
    .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
    .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
    .allianceRelativeControl(true);

// Derive the heading axis with math!
SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
    .withControllerHeadingAxis(
    () -> Math.sin(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
    () -> Math.cos(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
    .headingWhile(true);
/**
* The container for the robot. Contains subsystems, OI devices, and commands.
*/
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }
  // drive input --> field relative chassis speeds
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
    // Swerve drive controls
    Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngleKeyboard = m_swerveSubsystem.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAngularVelocityKeyboard = m_swerveSubsystem.driveFieldOriented(driveAngularVelocityKeyboard);
    // Default to field-centric swerve drive
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
    // Operator control
    m_operatorController.back().whileTrue(m_swerveSubsystem.centerModulesCommand());

    //left trigger bindings
    m_driverController.leftTrigger(0.05).whileTrue(new StrafeLeftCommand());
    m_driverController.rightTrigger(0.05).whileTrue(new StrafeRightCommand());
  }

  public void setMotorBrake(boolean brake) {
    m_swerveSubsystem.setMotorBrake(brake);
  }
}
