package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;

@SuppressWarnings("static-access")
public class RobotOrientedDrive extends Command {
	public static final double DEADBAND = 0.10; // FIXME. controller deadband
	public static final double TURN_DEADBAND = 0.10; // FIXME. controller turn deadband

	public RobotOrientedDrive() {
		addRequirements(drive);
	}

	public ChassisSpeeds run(double vx, double vy, double w) {
		double mag = hypot(vx, vy);
		double v = Util.squareCurve(Util.deadband(mag, DEADBAND));
		double wc = Util.squareCurve(Util.deadband(w, TURN_DEADBAND));

		double theta = atan2(vy, vx);

		double speedMul = 1; // FIXME. speed multiplier/limiter (tune)
		double wMul = 1; // FIXME. angular speed multiplier/limiter (tune)

		return new ChassisSpeeds(
				v * cos(theta) * drive.MAX_VEL * speedMul,
				v * sin(theta) * drive.MAX_VEL * speedMul,
				wc * drive.MAX_W * wMul);
	}

	@Override
	public void execute() {
		ChassisSpeeds speeds = run(controls.leftY(), controls.leftX(), controls.rightX());
		drive.drive(speeds);
	}
}
