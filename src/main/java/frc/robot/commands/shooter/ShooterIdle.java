package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterIdle extends Command {
	public ShooterIdle() {
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		shooter.targetFlywheelRPM = 1000;
		// shooter.targetFlywheelRPM = 0;
		// shooter.targetTurretPos = -45 * PI / 180.0;
		// shooter.targetTurretPos = -45 * PI / 180.0;
		// shooter.targetHoodPos = 20 * PI / 180.0;
	}
}
