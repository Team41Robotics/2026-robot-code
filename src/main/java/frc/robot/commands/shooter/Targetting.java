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

	public static final double[][] SHOT_TABLE = {
		// {dist (m), rpm, hood (rad), tof (s)}
		{1.5, 1530, 0.0, 0.948},
		{2.0, 1690, 0.0, 1.083},
		{2.25, 1730, 0.0, 1.159},
		{2.5, 1770, 0.0, 1.232},
		{2.75, 1810, 0.0113, 1.257},
		{3.0, 1860, 0.0227, 1.277},
		{3.29, 1910, 0.0359, 1.301}, // anchor
		{3.5, 1950, 0.0454, 1.339},
		{3.75, 2000, 0.0567, 1.349},
		{4.0, 2050, 0.0681, 1.357},
		{4.25, 2050, 0.0794, 1.395},
		{4.5, 2100, 0.0908, 1.398},
		{4.75, 2100, 0.1021, 1.432},
		{5.0, 2150, 0.1134, 1.432},
		{5.5, 2200, 0.1361, 1.460},
		{6.0, 2250, 0.1588, 1.483},
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
