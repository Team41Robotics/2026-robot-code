package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("static-access")
public class ShootOnTheFly extends Command {
	public Translation2d origTarget;

	public static final double DEADBAND = 0.10; // TUNEME. controller deadband
	public static final double TURN_DEADBAND = 0.50; // TUNEME. controller turn deadband

	public final ProfiledPIDController headingController;

	public ShootOnTheFly(Translation2d target) {
		this.origTarget = target;
		headingController = new ProfiledPIDController(5, 0, 0, new Constraints(drive.MAX_W, drive.MAX_W)); // TUNEME
		headingController.enableContinuousInput(-PI, PI);
		addRequirements(shooter, drive);
	}

	@Override
	public void initialize() {
		origTarget = Util.flipIfRed(origTarget);
		headingController.reset(drive.pose.getRotation().getRadians());
		headingController.setGoal(drive.pose.getRotation().getRadians());
	}

	@Override
	public void execute() {
		Translation2d target = Targetting.shootOnTheFly(origTarget);
		Rotation2d targetAngle = target.minus(drive.pose.getTranslation()).getAngle();
		headingController.setGoal(targetAngle.getRadians());

		double omega = headingController.calculate(drive.pose.getRotation().getRadians());

		double xSpeed = Util.squareCurve(Util.deadband(controls.leftX(), DEADBAND)) * drive.MAX_VEL;
		double ySpeed = Util.squareCurve(Util.deadband(controls.leftY(), DEADBAND)) * drive.MAX_VEL;

		ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, drive.rot);
		drive.drive(speeds);

		double distance = Targetting.shooterToTarget(target);
		double targetRPM = Targetting.FLYWHEELRPM_MAP.get(distance);
		shooter.targetFlywheelRPM = targetRPM;

		Logger.recordOutput("/Targetting/targetPose", target);
		Logger.recordOutput("/Targetting/distance", distance);
		field.getObject("shootTarget").setPose(new Pose2d(target, new Rotation2d()));
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		shooter.targetFlywheelRPM = 0;
	}
}
