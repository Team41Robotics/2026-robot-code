package frc.robot.commands.drive;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("static-access")
public class FieldHeadingDrive extends Command {
	public static double DEADBAND = 0.10;
	public static double TURN_DEADBAND = 0.50;

	public TrapezoidProfile.Constraints ROT_CONSTRAINTS =
			new TrapezoidProfile.Constraints(drive.MAX_OMEGA, drive.MAX_OMEGA / 1);

	public TrapezoidProfile profile = new TrapezoidProfile(ROT_CONSTRAINTS);
	public PIDController pid = new PIDController(1, 0, 0);

	public TrapezoidProfile.State setpointHeading = new TrapezoidProfile.State();

	public FieldHeadingDrive() {
		SmartDashboard.putData("PID", pid);
		addRequirements(drive);
		pid.enableContinuousInput(-PI, PI);

		Rotation2d heading = drive.pose.getRotation();
		if (isRed()) heading = heading.plus(Rotation2d.kPi);
		setpointHeading = new TrapezoidProfile.State(heading.getRadians(), 0);
	}

	public ChassisSpeeds run(double vx, double vy, double tx, double ty) {
		double mag = hypot(vx, vy);
		double mag_curved = Util.squareCurve(Util.deadband(mag, DEADBAND));

		// TODO: maybe angle snap?
		double theta = atan2(vy, vx);

		double speed_mul = 1;

		Rotation2d heading = drive.pose.getRotation();
		if (isRed()) heading = heading.plus(Rotation2d.kPi);

		double turn_theta = hypot(tx, ty) < TURN_DEADBAND ? setpointHeading.position : atan2(ty, tx);
		turn_theta = inputModulus(turn_theta - setpointHeading.position, 0, 2 * PI) + setpointHeading.position;

		TrapezoidProfile.State newHeading1 =
				profile.calculate(LOOP_PERIOD, setpointHeading, new TrapezoidProfile.State(turn_theta, 0));
		double time1 = profile.totalTime();
		turn_theta -= 2 * PI;
		TrapezoidProfile.State newHeading2 =
				profile.calculate(LOOP_PERIOD, setpointHeading, new TrapezoidProfile.State(turn_theta, 0));
		double time2 = profile.totalTime();

		TrapezoidProfile.State newHeading = time1 < time2 ? newHeading1 : newHeading2;
		setpointHeading = newHeading;

		ChassisSpeeds speeds = new ChassisSpeeds(
				mag_curved * cos(theta) * drive.MAX_VEL * speed_mul,
				mag_curved * sin(theta) * drive.MAX_VEL * speed_mul,
				pid.calculate(heading.getRadians(), newHeading.position) + newHeading.velocity);

		Logger.recordOutput("/ASDF/turn_theta", angleModulus(turn_theta));
		Logger.recordOutput("/ASDF/turn_setpoint", angleModulus(setpointHeading.position));
		Logger.recordOutput("/Error", pid.getError());

		return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading);
	}

	@Override
	public void execute() {
		ChassisSpeeds speeds = run(-left_js.getY(), -left_js.getX(), -right_js.getY(), -right_js.getX());
		drive.drive(speeds);
	}
}
