package frc.robot.commands.climber;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberUp extends Command {
	public static final double CLIMBER_UP_VOLTAGE = 5; // TUNEME

	public boolean released;

	public ClimberUp() {
		addRequirements(climber);
	}

	@Override
	public void initialize() {
		released = false;
		climber.targetVoltage = CLIMBER_UP_VOLTAGE;
	}

	@Override
	public void execute() {
		if (!climber.inputs.limit) released = true;
	}

	@Override
	public boolean isFinished() {
		return released && climber.inputs.limit;
	}

	@Override
	public void end(boolean interrupted) {
		climber.targetVoltage = 0;
	}
}
