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

	public static double flywhheelRPMtemp = 2000;

	public static ShotParameters shotSpeeds(double distance) {
		// double flywheelRPM = flywhheelRPMtemp; // TODO
		double flywheelRPM = 370 * distance + 1041;
		double hoodAngle = 0; // TODO
		double timeOfFlight = 0; // TODO
		return new ShotParameters(flywheelRPM, hoodAngle, timeOfFlight, distance);
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
