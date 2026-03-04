package frc.robot.commands.shooter;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;
import frc.robot.commands.shooter.Targetting.ShotParameters;
import org.littletonrobotics.junction.Logger;

public class ShootTeleop extends Command {
	public Translation2d origTarget, target;

	public ShootTeleop() {
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		target = Util.flipIfRed(origTarget);
	}

	@Override
	public void execute() {
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
		field.getObject("shootTarget").setPose(new Pose2d(virtualTarget, new Rotation2d()));
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
