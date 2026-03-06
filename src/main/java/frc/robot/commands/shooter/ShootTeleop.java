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
	public static final double JOYSTICK_SCALE = 1.0; // meters per full joystick deflection

	public ShootTeleop() {
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		Translation2d hubCenter = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
		Translation2d offset =
				new Translation2d(controls.thirdY() * JOYSTICK_SCALE, controls.thirdX() * JOYSTICK_SCALE);
		Translation2d target = Util.flipIfRed(hubCenter.plus(offset));

		Translation2d virtualTarget = Targetting.shootOnTheFly(target);
		double fieldAngle = Targetting.shotAngle(virtualTarget);
		shooter.targetTurretPos =
				angleModulus(fieldAngle - drive.pose.getRotation().getRadians() + PI);
		shooter.targetTurretVel = -drive.measuredSpeeds.omegaRadiansPerSecond;

		double distance = Targetting.targetRelative(virtualTarget).getNorm();
		ShotParameters params = Targetting.shotSpeeds(distance);
		shooter.targetFlywheelRPM = params.flywheelRPM();
		shooter.targetHoodPos = params.hoodAngle();

		Logger.recordOutput("/Targetting/targetPose", virtualTarget);
		Logger.recordOutput("/Targetting/joystickTarget", target);
		Logger.recordOutput("/Targetting/distance", distance);
		field.getObject("shootTarget").setPose(new Pose2d(virtualTarget, new Rotation2d()));
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
