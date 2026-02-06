package frc.robot.test.drive;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class DrivePIDTestCommand extends Command {
	public DrivePIDTestCommand() {
		addRequirements(drive);
	}

	@Override
	public void execute() {
		for (int i = 0; i < drive.modules.length; i++) {
			drive.modules[i].drive(
					new SwerveModuleState(2 * (left_js.getHID().getRawButton(1) ? -1 : 1), new Rotation2d()));
		}
	}
}
