package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterIdle extends Command {
	public ShooterIdle() {
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		shooter.targetFlywheelRPM = 2000;
	}
}
