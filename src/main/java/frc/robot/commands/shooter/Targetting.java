package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Targetting {

	public static final InterpolatingDoubleTreeMap FLYWHEELRPM_MAP = new InterpolatingDoubleTreeMap();
	public static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_MAP = new InterpolatingDoubleTreeMap();

	public static void loadData() {
		//Key: distance to target, Value: flywheel RPM 
		FLYWHEELRPM_MAP.put(1.0, 1650.0);
		//Key: distance to target, Value: time of flight
		TIME_OF_FLIGHT_MAP.put(1.0, 0.5);
	}
	
	/*public static Translation2d targetOnTheFly(Translation2d target, double timeOfFlight) {
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
	}*/
}
