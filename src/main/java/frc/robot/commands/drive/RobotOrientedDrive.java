package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;

@SuppressWarnings("static-access")
public class RobotOrientedDrive extends Command {
	public static double DEADBAND = 0.10;
	public static double TURN_DEADBAND = 0.10;

	public RobotOrientedDrive() {
		addRequirements(drive);
	}

	public ChassisSpeeds run(double vx, double vy, double w) {
		double mag = hypot(vx, vy);
		double mag_curved = Util.squareCurve(Util.deadband(mag, DEADBAND));
		double w_curved = Util.squareCurve(Util.deadband(w, TURN_DEADBAND));

		// TODO: maybe angle snap?
		double theta = atan2(vy, vx);

		double speed_mul = 1;
		double angular_speed_mul = 1;

		return new ChassisSpeeds(
				mag_curved * cos(theta) * drive.MAX_VEL * speed_mul,
				mag_curved * sin(theta) * drive.MAX_VEL * speed_mul,
				w_curved * drive.MAX_OMEGA * angular_speed_mul);
	}

	@Override
	public void execute() {
		ChassisSpeeds speeds = run(-xbox.getLeftY(), -xbox.getLeftX(), -xbox.getRightX());
		drive.drive(speeds);
	}
}
