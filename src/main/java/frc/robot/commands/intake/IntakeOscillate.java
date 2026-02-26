package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeOscillate extends Command {
	public static double HIGH_VOLTAGE = 10.0; // FIXME. high intake voltage (V)
	public static double CENTER_POSITION = 0.5; // FIXME. oscillation center position (radians)
	public static double OSCILLATE_AMPLITUDE = 0.2; // FIXME. oscillation amplitude (radians)
	public static double OSCILLATE_PERIOD = 0.5; // FIXME. oscillation period (seconds)

	public double startTime;

	public IntakeOscillate() {
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		startTime = Timer.getFPGATimestamp();
		intake.setIntakeVoltage(HIGH_VOLTAGE);
	}

	@Override
	public void execute() {
		double elapsed = Timer.getFPGATimestamp() - startTime;
		double center = CENTER_POSITION;
		double offset = OSCILLATE_AMPLITUDE * sin(2.0 * PI * elapsed / OSCILLATE_PERIOD);
		intake.setJointPosition(center + offset);
	}

	@Override
	public boolean isFinished() {
		return false; // Runs until interrupted
	}

	@Override
	public void end(boolean interrupted) {
		intake.setIntakeVoltage(0);
	}
}
