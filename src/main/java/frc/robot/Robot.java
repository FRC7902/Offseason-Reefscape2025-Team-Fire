
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.MoveElevatorArmCommand;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    private final RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    public Robot() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        autoFactory = new AutoFactory(
                RobotContainer.m_swerveSubsystem::getPose, // A function that returns the current robot pose
                RobotContainer.m_swerveSubsystem::resetOdometry, // A function that resets the current robot pose to the
                                                                 // provided Pose2d
                RobotContainer.m_swerveSubsystem::followTrajectory, // The drive subsystem trajectory follower
                true, // If alliance flipping should be enabled
                RobotContainer.m_swerveSubsystem // The drive subsystem
        );

        // Create the auto chooser
        autoChooser = new AutoChooser();

        // Add options to the chooser
        autoChooser.addRoutine("Test Routine", this::testAuto);
        // autoChooser.addCmd("Example Auto Command", this::exampleAutoCommand);

        // Put the auto chooser on the dashboard
        SmartDashboard.putData("autoChooser", autoChooser);

        // Schedule the selected auto during the autonomous period
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    }

    public AutoRoutine testAuto() {
        AutoRoutine routine = autoFactory.newRoutine("testAuto");

        // Load the routine's trajectories
        AutoTrajectory startToM1 = routine.trajectory("startToM1");
        AutoTrajectory M1toM2 = routine.trajectory("M1toM2");

        routine.active().onTrue(
                Commands.sequence(
                        startToM1.resetOdometry(),
                        startToM1.cmd(),
                        // new MoveElevatorArmCommand(ElevatorPosition.REST),
                        M1toM2.cmd()));

        return routine;
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // m_robotContainer.setMotorBrake(true);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        // m_robotContainer.setMotorBrake(true);

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}