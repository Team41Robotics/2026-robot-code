package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterRest extends Command {
	public ShooterRest() {
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		shooter.stop();
	}

	@Override
	public void execute() {}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {}
}
