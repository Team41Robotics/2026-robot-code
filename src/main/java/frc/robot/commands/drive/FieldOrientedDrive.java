package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class FieldOrientedDrive extends Command {
	public FieldOrientedDrive() {
		addRequirements(drive.subsystem);
	}

	@Override
	public void execute() {
		ChassisSpeeds speeds = RobotOrientedDrive.run(left_js.getY(), left_js.getX(), -right_js.getX());
		drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.pose.getRotation())); // TODO
	}
}
