package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;

public class ManualShoot extends Command {
	public static final double DEADBAND = 0.10;
	public static final double HOOD_INCREMENT = 5 / 180. * PI;

	public boolean prevPovUp = false;
	public boolean prevPovDown = false;

	public ManualShoot() {
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		double lx = controls.leftX();
		double ly = controls.leftY();
		double mag = Util.deadband(hypot(lx, ly), DEADBAND);

		// shooter.targetFlywheelRPM = mag * 3000 + 1500;
		shooter.targetFlywheelRPM = 0;

		if (mag > 0) {
			// shooter.targetTurretPos = atan2(lx, ly);
			shooter.targetTurretPos = shooter.TURRET_POS_MIN
					+ (shooter.TURRET_POS_MAX - shooter.TURRET_POS_MIN) * (sin(Timer.getTimestamp()) + 1) / 2.;
		}

		boolean povUp = controls.pov(0).getAsBoolean();
		boolean povDown = controls.pov(180).getAsBoolean();

		if (povUp && !prevPovUp) shooter.targetHoodPos += HOOD_INCREMENT;
		else if (povDown && !prevPovDown) shooter.targetHoodPos -= HOOD_INCREMENT;

		prevPovUp = povUp;
		prevPovDown = povDown;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
