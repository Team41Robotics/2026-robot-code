package frc.robot.test.drive;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DrivePIDTestCommand extends Command {
	public DrivePIDTestCommand() {
		addRequirements(drive.subsystem);
	}

	@Override
	public void execute() {
		for (int i = 0; i < drive.modules.length; i++) {
			int t = (int) floor(Timer.getTimestamp());
			drive.modules[i].drive(new SwerveModuleState(t % 2 == 0 ? t : -t, new Rotation2d()));
		}
	}
}
