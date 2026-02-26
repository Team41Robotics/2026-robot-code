package frc.robot.commands.autos;

import static java.lang.Math.*;
import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;

public class ResetPose extends Command {
	public Pose2d pose;

	public ResetPose(double x, double y, double theta) {
		addRequirements(drive);
		this.pose = new Pose2d(x, y, new Rotation2d(theta));
	}

	public void flip() {
		this.pose = new Pose2d(
				FieldConstants.fieldWidth - pose.getX(),
				FieldConstants.fieldLength - pose.getY(),
				pose.getRotation().plus(new Rotation2d(PI)));
	}

	@Override
	public void initialize() {
		if (isRed()) {
			flip();
		}
		drive.resetPose(pose);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
