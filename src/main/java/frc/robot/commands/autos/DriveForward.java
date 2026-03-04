package frc.robot.commands.autos;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;

public class DriveForward extends Command {
	public double origX, origY, origTheta;
	public double x, y, theta;
	public double dist, speed, maxTime;
	public double startTime;

	public DriveForward(double x, double y, double xf, double yf, double speed, double maxTime) {
		addRequirements(drive);
		this.origX = x;
		this.origY = y;
		this.origTheta = atan2(yf - y, xf - x);
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

	@Override
	public void initialize() {
		this.theta = Util.flipIfRed(origTheta);
		Translation2d flipped = Util.flipIfRed(new Translation2d(origX, origY));
		this.x = flipped.getX();
		this.y = flipped.getY();
		startTime = Timer.getTimestamp();
	}

	@Override
	public void execute() {
		drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speed * cos(theta), speed * sin(theta), 0, drive.rot));
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
