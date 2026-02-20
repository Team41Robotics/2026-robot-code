package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterActive extends Command {
	public static final double SHOOTER_RPM = 3500; // FIXME TODO

	public ShooterActive() {
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		shooter.setVelocity(SHOOTER_RPM);
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
