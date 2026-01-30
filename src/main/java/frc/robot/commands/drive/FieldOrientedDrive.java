package frc.robot.commands.drive;

import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.left_js;
import static frc.robot.RobotContainer.right_js;
import static java.lang.Math.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class FieldOrientedDrive extends Command {
	public FieldOrientedDrive() {
		addRequirements(drive);
	}

	@Override
	public void execute() {
		ChassisSpeeds speeds = RobotOrientedDrive.run(left_js.getY(), left_js.getX(), -right_js.getX());
		drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, null)); // TODO
	}
}
