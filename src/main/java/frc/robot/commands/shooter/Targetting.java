package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Targetting {
	public static final Transform2d TURRET_POS = new Transform2d(new Translation2d(0.174, 0.159), new Rotation2d(0));

	public record ShotParameters(double flywheelRPM, double hoodAngle, double timeOfFlight, double distance) {}

	public static Translation2d targetRelative(Translation2d target) {
		Pose2d turretPos = drive.pose.plus(TURRET_POS);
		return target.minus(turretPos.getTranslation());
	}

	public static double shotAngle(Translation2d target) {
		Translation2d toTarget = targetRelative(target);
		return toTarget.getAngle().getRadians();
	}

	// {distance (m), flywheelRPM, hoodAngle (rad), timeOfFlight (s)}
	public static final double[][] SHOT_TABLE = {
		// dist,  rpm,   hood,  tof
		{1.0, 1650, 0, 0.83},
		{1.5, 1700, 0, 0.87},
		{2.0, 1750, 0, 0.936},
		{2.5, 1800, 0, 1.137},
		{3.0, 1850, 0.0175, 1.262},
		{3.29, 1900, 0.0349, 1.284},
		{3.5, 1950, 0.0524, 1.271},
		{4.0, 2000, 0.0698, 1.356},
		{4.5, 2050, 0.0873, 1.428},
		{5, 2100, 0.1047, 1.49},
		{5.5, 2150, 0.1396, 1.487},
		{6, 2250, 0.1571, 1.498}
	};

	public static ShotParameters shotSpeeds(double distance) {
		// extrapolate below table
		if (distance <= SHOT_TABLE[0][0]) {
			double[] lo = SHOT_TABLE[0];
			double[] hi = SHOT_TABLE[1];
			double t = (distance - lo[0]) / (hi[0] - lo[0]);
			return new ShotParameters(
					lo[1] + t * (hi[1] - lo[1]), lo[2] + t * (hi[2] - lo[2]), lo[3] + t * (hi[3] - lo[3]), distance);
		}
		// extrapolate above table
		if (distance >= SHOT_TABLE[SHOT_TABLE.length - 1][0]) {
			double[] lo = SHOT_TABLE[SHOT_TABLE.length - 2];
			double[] hi = SHOT_TABLE[SHOT_TABLE.length - 1];
			double t = (distance - lo[0]) / (hi[0] - lo[0]);
			return new ShotParameters(
					lo[1] + t * (hi[1] - lo[1]), lo[2] + t * (hi[2] - lo[2]), lo[3] + t * (hi[3] - lo[3]), distance);
		}

		// linear interpolation between bracketing rows
		for (int i = 0; i < SHOT_TABLE.length - 1; i++) {
			double[] lo = SHOT_TABLE[i];
			double[] hi = SHOT_TABLE[i + 1];
			if (distance >= lo[0] && distance <= hi[0]) {
				double t = (distance - lo[0]) / (hi[0] - lo[0]);
				double rpm = lo[1] + t * (hi[1] - lo[1]);
				double hood = lo[2] + t * (hi[2] - lo[2]);
				double tof = lo[3] + t * (hi[3] - lo[3]);
				return new ShotParameters(rpm, hood, tof, distance);
			}
		}

		// fallback (should never reach here)
		return new ShotParameters(0, 0, 0, distance);
	}

	public static Translation2d targetOnTheFly(Translation2d target, double timeOfFlight) {
		ChassisSpeeds speeds = drive.measuredSpeeds;
		double omega = speeds.omegaRadiansPerSecond;
		Translation2d turretOffset = TURRET_POS.getTranslation();

		// Turret velocity in robot frame = robot center velocity + omega cross turret offset
		double vx = speeds.vxMetersPerSecond - omega * turretOffset.getY();
		double vy = speeds.vyMetersPerSecond + omega * turretOffset.getX();

		// Convert to field frame
		Translation2d vel = new Translation2d(vx, vy).rotateBy(drive.rot);
		return target.minus(vel.times(timeOfFlight));
	}

	public static Translation2d shootOnTheFly(Translation2d target) {
		Translation2d newTarget = target;
		for (int i = 0; i < 5; i++) {
			ShotParameters params = shotSpeeds(targetRelative(newTarget).getNorm());
			newTarget = targetOnTheFly(target, params.timeOfFlight());
		}
		return newTarget;
	}
}
