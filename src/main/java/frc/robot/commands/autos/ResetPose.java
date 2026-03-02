package frc.robot.commands.autos;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;

public class ResetPose extends Command {
	public Pose3d pose;

	public ResetPose(double x, double y, double theta) {
		addRequirements(drive);
		this.pose = new Pose3d(x, y, 0, new Rotation3d(0, 0, theta));
	}

	public void flip() {
		this.pose = new Pose3d(
				FieldConstants.fieldWidth - pose.getX(),
				FieldConstants.fieldLength - pose.getY(),
				0,
				pose.getRotation().plus(new Rotation3d(0, 0, Math.PI)));
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
