package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Targetting {
	public static final Transform2d TURRET_POS =
			new Transform2d(new Translation2d(0.174, 0.159), new Rotation2d(0)); // TODO

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
		{1.0, 1400, 0.80, 0.15},
		{2.0, 1800, 0.60, 0.25},
		{3.0, 2200, 0.45, 0.35},
		{4.0, 2600, 0.35, 0.45},
		{5.0, 3000, 0.28, 0.55},
	};

	public static ShotParameters shotSpeeds(double distance) {
		// extrapolate below table
		if (distance <= SHOT_TABLE[0][0]) {
			double[] lo = SHOT_TABLE[0];
			double[] hi = SHOT_TABLE[1];
			double t = (distance - lo[0]) / (hi[0] - lo[0]);
			return new ShotParameters(
					lo[1] + t * (hi[1] - lo[1]),
					lo[2] + t * (hi[2] - lo[2]),
					lo[3] + t * (hi[3] - lo[3]),
					distance);
		}
		// extrapolate above table
		if (distance >= SHOT_TABLE[SHOT_TABLE.length - 1][0]) {
			double[] lo = SHOT_TABLE[SHOT_TABLE.length - 2];
			double[] hi = SHOT_TABLE[SHOT_TABLE.length - 1];
			double t = (distance - lo[0]) / (hi[0] - lo[0]);
			return new ShotParameters(
					lo[1] + t * (hi[1] - lo[1]),
					lo[2] + t * (hi[2] - lo[2]),
					lo[3] + t * (hi[3] - lo[3]),
					distance);
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
		Translation2d vel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
		vel = vel.rotateBy(drive.rot);
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
