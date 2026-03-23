package frc.robot.commands.shooter;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.Util;
import frc.robot.commands.shooter.Targetting.ShotParameters;
import org.littletonrobotics.junction.Logger;

public class ShootTeleop extends Command {
	public static final double JOYSTICK_SCALE = 0.3; // meters per full joystick deflection
	public static final double TRENCH_Y_OFFSET = 1.7; // meters offset when shooting over trench

	public ShootTeleop() {
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		Translation2d hubCenter = FieldConstants.Hub.innerCenterPoint.toTranslation2d();

		// Convert robot position to alliance-relative coordinates (blue perspective)
		Translation2d robotPos = Util.flipIfRed(drive.pose.getTranslation());
		double robotX = robotPos.getX();
		double robotY = robotPos.getY();
		double xTrenchMin = FieldConstants.LinesVertical.hubCenter - FieldConstants.LeftBump.width / 2.0;
		double xTrenchMax = FieldConstants.LinesVertical.hubCenter + FieldConstants.LeftBump.width / 2.0;
		double centerY = FieldConstants.fieldWidth / 2.0;

		Translation2d target;
		boolean overrideHood = false;
		String state;

		if (robotX < xTrenchMin) {
			// Before trench - shoot at hub with joystick offset
			Translation2d offset =
					new Translation2d(controls.thirdY() * JOYSTICK_SCALE, controls.thirdX() * JOYSTICK_SCALE);
			target = Util.flipIfRed(hubCenter.plus(offset));
			state = "SHOOT_HUB";
		} else if (robotX < xTrenchMax) {
			// In trench zone - hood to 0, aim at hub
			target = Util.flipIfRed(hubCenter);
			overrideHood = true;
			state = "TRENCH";
		} else if (controls.passToOwnSide().getAsBoolean()) {
			// Past trench with pass button held - pass to left or right side, NOT through middle
			double yOffset = (robotY < centerY) ? -TRENCH_Y_OFFSET : TRENCH_Y_OFFSET;
			target = Util.flipIfRed(hubCenter.plus(new Translation2d(0, yOffset)));
			state = "PASS";
		} else {
			// Past trench without pass button - shoot at hub with joystick offset
			Translation2d offset =
					new Translation2d(controls.thirdY() * JOYSTICK_SCALE, controls.thirdX() * JOYSTICK_SCALE);
			target = Util.flipIfRed(hubCenter.plus(offset));
			state = "SHOOT_HUB";
		}

		Translation2d virtualTarget = Targetting.shootOnTheFly(target);
		double fieldAngle = Targetting.shotAngle(virtualTarget);
		shooter.targetTurretPos =
				angleModulus(fieldAngle - drive.pose.getRotation().getRadians() + PI);

		double distance = Targetting.targetRelative(virtualTarget).getNorm();
		ShotParameters params = Targetting.shotSpeeds(distance);
		shooter.targetFlywheelRPM = params.flywheelRPM();
		shooter.targetHoodPos = overrideHood ? 0 : params.hoodAngle();

		Logger.recordOutput("/Targetting/state", state);
		Logger.recordOutput("/Targetting/targetPose", new Pose2d(virtualTarget, new Rotation2d()));
		Logger.recordOutput("/Targetting/joystickTarget", new Pose2d(target, new Rotation2d()));
		Logger.recordOutput("/Targetting/distance", distance);
		field.getObject("shootTarget").setPose(new Pose2d(virtualTarget, new Rotation2d()));

		// Zone boundary lines (vertical lines spanning field width)
		double fieldW = FieldConstants.fieldWidth;
		Pose2d[] trenchMinLine = new Pose2d[] {
			new Pose2d(xTrenchMin, 0, new Rotation2d()), new Pose2d(xTrenchMin, fieldW, new Rotation2d())
		};
		Pose2d[] trenchMaxLine = new Pose2d[] {
			new Pose2d(xTrenchMax, 0, new Rotation2d()), new Pose2d(xTrenchMax, fieldW, new Rotation2d())
		};
		Logger.recordOutput("/Targetting/trenchMinLine", trenchMinLine);
		Logger.recordOutput("/Targetting/trenchMaxLine", trenchMaxLine);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
