package frc.robot.commands.shooter;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;

public class ShooterTargetting extends Command {
	public static final Transform2d TURRET_POS =
			new Transform2d(new Translation2d(0.174, 0.159), new Rotation2d(0)); // TODO

	public ShooterTargetting() {
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		Translation2d target = FieldConstants.Hub.topCenterPoint.toTranslation2d();

		Pose2d turretPos = drive.pose.plus(TURRET_POS);

		Translation2d toTarget = target.minus(turretPos.getTranslation());
		double angleToTarget = atan2(toTarget.getY(), toTarget.getX());
		shooter.targetTurretPos = angleModulus(angleToTarget + PI);
		shooter.targetTurretVel = -drive.measuredSpeeds.omegaRadiansPerSecond;
	}
}
