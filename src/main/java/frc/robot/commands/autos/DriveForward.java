package frc.robot.commands.autos;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;

@SuppressWarnings("static-access")
public class DriveForward extends Command {
	public double x, y, theta, dist, speed, maxTime;
	public double startTime;

	public DriveForward(double x, double y, double xf, double yf, double speed, double maxTime) {
		addRequirements(drive);
		this.x = x;
		this.y = y;
		this.theta = atan2(yf - y, xf - x);
		this.dist = hypot(xf - x, yf - y);
		this.maxTime = maxTime;
		this.speed = speed;
	}

	public DriveForward(double x, double y, double xf, double yf, double speed) {
		this(x, y, xf, yf, speed, Double.POSITIVE_INFINITY);
	}

	public DriveForward(double x, double y, double xf, double yf) {
		this(x, y, xf, yf, 0.5);
	}

	public void flip() {
		this.theta += PI;
		this.x = FieldConstants.fieldWidth - x;
		this.y = FieldConstants.fieldLength - y;
	}

	@Override
	public void initialize() {
		if (isRed()) {
			flip();
		}
		startTime = Timer.getTimestamp();
	}

	@Override
	public void execute() {
		drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
				speed * cos(theta), speed * sin(theta), 0, drive.pose.getRotation()));
	}

	@Override
	public boolean isFinished() {
		return (drive.pose.getX() - x) * cos(theta) + (drive.pose.getY() - y) * sin(theta) >= dist
				|| Timer.getTimestamp() - startTime >= maxTime;
	}

	@Override
	public void end(boolean interrupted) {
		drive.drive(new ChassisSpeeds());
	}
}
