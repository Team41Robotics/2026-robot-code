package frc.robot.commands.climber;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDown extends Command {
	public static final double CLIMBER_DOWN_VOLTAGE = -5; // TUNEME

	public ClimberDown() {
		addRequirements(climber);
	}

	@Override
	public void initialize() {
		climber.targetVoltage = CLIMBER_DOWN_VOLTAGE;
	}

	@Override
	public boolean isFinished() {
		return climber.inputs.limitBottom;
	}

	@Override
	public void end(boolean interrupted) {
		climber.targetVoltage = 0;
	}
}
