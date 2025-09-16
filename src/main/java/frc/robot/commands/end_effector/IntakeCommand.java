// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.end_effector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {

    public enum IntakeMode {
        ALGAE,
        CORAL
    }

    private IntakeMode m_mode;

    Debouncer m_debouncer;

    /**
     * Creates a new IntakeAlgaeCoralCommand.
     */
    public IntakeCommand(IntakeMode mode) {
        addRequirements(RobotContainer.m_endEffectorSubsystem);

        m_mode = mode;

        // TODO: Tune debouncer time
        m_debouncer = new Debouncer(0.33, DebounceType.kRising);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_debouncer.calculate(false);

        RobotContainer.m_endEffectorSubsystem.setSpeed(EndEffectorConstants.INTAKE_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Applies a continuous holding speed to the game piece until the OuttakeCommand is run, which then the motors will stop
        if (m_mode == IntakeMode.CORAL) {
            RobotContainer.m_endEffectorSubsystem.setSpeed(EndEffectorConstants.CORAL_HOLD_SPEED);
        } else {
            // TODO: Tune if 1.0 speed is necessary
            RobotContainer.m_endEffectorSubsystem.setSpeed(EndEffectorConstants.ALGAE_HOLD_SPEED);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Coral mode
        if (m_mode == IntakeMode.CORAL) {
            return RobotContainer.m_endEffectorSubsystem.hasCoral();
        }

        // Algae mode
        // Debouncer to detect consistent current spike for longer than time (t)
        return m_debouncer.calculate(
                RobotContainer.m_endEffectorSubsystem.getSupplyCurrent() > EndEffectorConstants.ALGAE_INTAKE_STALL_DETECTION_CURRENT
        );

        // TODO: Check supply current readings on AdvantageScope?
        // TODO: Maybe filter current through LinearFilter.movingAverage(10)?
        // TODO: Adjust stall detection current?
    }
}
