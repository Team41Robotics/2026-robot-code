package frc.robot.commands.climber;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class PrepareClimb extends Command {
	public static final double ACTUATOR_VOLTAGE = 5; // TUNEME

	public PrepareClimb() {
		addRequirements(climber);
	}

	@Override
	public void initialize() {
		climber.actuatorTargetVoltage = ACTUATOR_VOLTAGE;
	}

	@Override
	public boolean isFinished() {
		// return climber.inputs.limitActuator;
		return false; // TODO
	}

	@Override
	public void end(boolean interrupted) {
		climber.actuatorTargetVoltage = 0;
	}
}
