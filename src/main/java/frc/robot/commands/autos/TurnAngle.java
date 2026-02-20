package frc.robot.commands.autos;

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

@SuppressWarnings("static-access")
public class TurnAngle extends Command {
	public static final double ANGLE_TOLERANCE = toRadians(2); // FIXME. finish tolerance (rad)
	public static final double TURN_VEL_TOLERANCE = 0.05; // FIXME. turn velocity finish tolerance (rad/s)

	public static final Constraints ROT_CONSTRAINTS =
			new Constraints(drive.MAX_W, drive.MAX_W); // FIXME. turn constraints

	public TrapezoidProfile profile = new TrapezoidProfile(ROT_CONSTRAINTS);
	public PIDController pid = new PIDController(5, 0, 0); // FIXME. heading PID (P,I,D)

	public State setpointHeading = new State();
	public double targetAngle;

	public TurnAngle(double targetAngle) {
		addRequirements(drive);
		this.targetAngle = targetAngle;
		pid.enableContinuousInput(-PI, PI);
	}

	@Override
	public void initialize() {
		setpointHeading = new State(drive.pose.getRotation().getRadians(), 0);
	}

	@Override
	public void execute() {
		Rotation2d heading = drive.pose.getRotation();

		double targetTheta = targetAngle;

		targetTheta = inputModulus(targetTheta - setpointHeading.position, 0, 2 * PI) + setpointHeading.position;
		State h1 = profile.calculate(LOOP_PERIOD, setpointHeading, new State(targetTheta, 0));
		double t1 = profile.totalTime();

		targetTheta -= 2 * PI;
		State h2 = profile.calculate(LOOP_PERIOD, setpointHeading, new State(targetTheta, 0));
		double t2 = profile.totalTime();

		setpointHeading = t1 < t2 ? h1 : h2;

		double omega = pid.calculate(heading.getRadians(), setpointHeading.position) + setpointHeading.velocity;
		drive.drive(new ChassisSpeeds(0, 0, omega));
	}

	@Override
	public boolean isFinished() {
		return abs(pid.getError()) < ANGLE_TOLERANCE && abs(pid.getErrorDerivative()) < TURN_VEL_TOLERANCE;
	}

	@Override
	public void end(boolean interrupted) {
		drive.drive(new ChassisSpeeds());
	}
}
