package frc.robot.commands.autos;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;

public class ResetPose extends Command {
	public Pose2d pose;

	public ResetPose(double x, double y, double theta) {
		addRequirements(drive);
		this.pose = new Pose2d(x, y, new Rotation2d(theta));
	}

	@Override
	public void initialize() {
		pose = Util.flipIfRed(pose);
		drive.resetPose(pose);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
