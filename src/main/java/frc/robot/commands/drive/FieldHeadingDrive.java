package frc.robot.commands.drive;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;
import org.littletonrobotics.junction.Logger;

public class FieldHeadingDrive extends Command {
	public static double DEADBAND = 0.10;
	public static double TURN_DEADBAND = 0.10;

	public PIDController pid = new PIDController(6, 0, 0);

	public FieldHeadingDrive() {
		SmartDashboard.putData("PID", pid);
		addRequirements(drive);
		pid.enableContinuousInput(-PI, PI);
	}

	public ChassisSpeeds run(double vx, double vy, double tx, double ty) {
		double mag = hypot(vx, vy);
		double mag_curved = Util.squareCurve(Util.deadband(mag, DEADBAND));

		// TODO: maybe angle snap?
		double theta = atan2(vy, vx);

		double speed_mul = 1;
		double angular_speed_mul = 1;

		Rotation2d heading = drive.pose.getRotation();
		if (isRed()) heading = heading.plus(Rotation2d.kPi);

		double turn_theta = hypot(tx, ty) < DEADBAND ? heading.getRadians() : atan2(ty, tx);

		ChassisSpeeds speeds = new ChassisSpeeds(
				mag_curved * cos(theta) * drive.MAX_VEL * speed_mul,
				mag_curved * sin(theta) * drive.MAX_VEL * speed_mul,
				pid.calculate(heading.getRadians(), turn_theta));

		Logger.recordOutput("/Error", pid.getError());

		return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading);
	}

	@Override
	public void execute() {
		ChassisSpeeds speeds = run(left_js.getY(), left_js.getX(), -right_js.getY(), -right_js.getX());
		drive.drive(speeds);
	}
}
