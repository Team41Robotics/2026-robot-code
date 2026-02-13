package frc.robot.commands.drive;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;

@SuppressWarnings("static-access")
public class FieldSnakeDrive extends Command {
	public static final double DEADBAND = 0.10; // FIXME. controller deadband
	public static final double TURN_DEADBAND = 0.50; // FIXME. controller turn deadband

	public static final Constraints ROT_CONSTRAINTS = new Constraints(drive.MAX_W, drive.MAX_W); // FIXME. turn constraints

	public TrapezoidProfile profile = new TrapezoidProfile(ROT_CONSTRAINTS);
	public PIDController pid = new PIDController(5, 0, 0); // FIXME. heading PID (P,I,D)

	public State setpointHeading = new State();

	public double targetTheta;

	public FieldSnakeDrive() {
		addRequirements(drive);
		pid.enableContinuousInput(-PI, PI);

		Rotation2d heading = drive.pose.getRotation();
		if (isRed()) heading = heading.plus(Rotation2d.kPi);
		setpointHeading = new State(heading.getRadians(), 0);
	}

	public ChassisSpeeds run(double vx, double vy, double tx, double ty) {
		double mag = hypot(vx, vy);
		double v = Util.squareCurve(Util.deadband(mag, DEADBAND));

		double theta = atan2(vy, vx);

		double speedMul = 1; // FIXME. speed multiplier/limiter (tune)

		Rotation2d heading = drive.pose.getRotation();
		if (isRed()) heading = heading.plus(Rotation2d.kPi);

		targetTheta = hypot(tx, ty) < TURN_DEADBAND ? (mag < TURN_DEADBAND ? targetTheta : theta) : atan2(ty, tx);

		targetTheta = inputModulus(targetTheta - setpointHeading.position, 0, 2 * PI) + setpointHeading.position;
		State h1 = profile.calculate(LOOP_PERIOD, setpointHeading, new State(targetTheta, 0));
		double t1 = profile.totalTime();

		targetTheta -= 2 * PI;
		State h2 = profile.calculate(LOOP_PERIOD, setpointHeading, new State(targetTheta, 0));
		double t2 = profile.totalTime();

		State newHeading = t1 < t2 ? h1 : h2;
		setpointHeading = newHeading;

		ChassisSpeeds speeds = new ChassisSpeeds(
				v * cos(theta) * drive.MAX_VEL * speedMul,
				v * sin(theta) * drive.MAX_VEL * speedMul,
				pid.calculate(heading.getRadians(), newHeading.position) + newHeading.velocity);

		return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading);
	}

	@Override
	public void execute() {
		ChassisSpeeds speeds = run(controls.leftY(), controls.leftX(), controls.rightY(), controls.rightX());
		drive.drive(speeds);
	}
}
