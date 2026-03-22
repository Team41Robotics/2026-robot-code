package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;

@SuppressWarnings("static-access")
public class ShootAtTarget extends Command {
	public static final double DEADBAND = 0.10; // TUNEME. controller deadband
	public static final double TURN_DEADBAND = 0.50; // TUNEME. controller turn deadband

	public static Translation2d target;
	public final ProfiledPIDController headingController;

	public ShootAtTarget(Translation2d target) {
		this.target = target;
		headingController = new ProfiledPIDController(5, 0, 0, new Constraints(drive.MAX_W, drive.MAX_W)); // TUNEME
		headingController.enableContinuousInput(-PI, PI);
		addRequirements(shooter, drive);
	}

	@Override
	public void initialize() {
		target = Util.flipIfRed(target);
		headingController.reset(drive.pose.getRotation().getRadians());
		headingController.setGoal(drive.pose.getRotation().getRadians());
	}

	@Override
	public void execute() {
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
