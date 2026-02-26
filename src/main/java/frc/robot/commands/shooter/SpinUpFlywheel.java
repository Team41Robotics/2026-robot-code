package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class SpinUpFlywheel extends Command {
	public static final double DEFAULT_VEL = 400.0; // FIXME rad/s

	public double velocity;

	public SpinUpFlywheel() {
		this(DEFAULT_VEL);
	}

	public SpinUpFlywheel(double velocity) {
		this.velocity = velocity;
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		shooter.targetFlywheelVel = velocity;
	}

	@Override
	public void end(boolean interrupted) {
		shooter.targetFlywheelVel = 0;
	}
}
