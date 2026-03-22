package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Targetting {

	public static final InterpolatingDoubleTreeMap FLYWHEELRPM_MAP = new InterpolatingDoubleTreeMap();
	public static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_MAP = new InterpolatingDoubleTreeMap();

	public static void loadData() {
		// Key: distance to target, Value: flywheel RPM
		FLYWHEELRPM_MAP.put(1.0, 1650.0);
		// Key: distance to target, Value: time of flight
		TIME_OF_FLIGHT_MAP.put(1.0, 0.5);
	}

	public static Translation2d getShooterPosition() {
		return drive.pose.getTranslation().plus(new Translation2d(0.0, 0.0)); // TODO: add actual shooter offset
	}

	public static double shooterToTarget(Translation2d target) {
		return target.minus(getShooterPosition()).getNorm();
	}

	public static Translation2d shootOnTheFly(Translation2d target) {
		ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.measuredSpeeds, drive.rot);
		double timeOfFlight = TIME_OF_FLIGHT_MAP.get((shooterToTarget(target)));
		Translation2d virtualTarget = target;
		double distance = target.minus(getShooterPosition()).getNorm();
		for (int i = 0; i < 20; i++) {
			timeOfFlight = TIME_OF_FLIGHT_MAP.get(distance);
			double offSetX = timeOfFlight * fieldSpeeds.vxMetersPerSecond;
			double offSetY = timeOfFlight * fieldSpeeds.vyMetersPerSecond;
			virtualTarget = target.minus(new Translation2d(offSetX, offSetY));
			distance = virtualTarget.minus(getShooterPosition()).getNorm();
		}
		return virtualTarget;
	}
}
