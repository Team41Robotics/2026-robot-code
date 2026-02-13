package frc.robot.test.drive;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DrivePIDTestCommand extends Command {
	public DrivePIDTestCommand() {
		addRequirements(drive);
	}

	double velocity = 2;

	@Override
	public void initialize() {
		ctrl.sysidQuasiForward().onChange(new InstantCommand(() -> {
			velocity = -velocity;
		}));
	}

	@Override
	public void execute() {
		for (int i = 0; i < drive.modules.length; i++) {
			drive.modules[i].drive(
					new SwerveModuleState(ctrl.sysidQuasiBackward().getAsBoolean() ? velocity : 0, new Rotation2d()));
		}
	}
}
