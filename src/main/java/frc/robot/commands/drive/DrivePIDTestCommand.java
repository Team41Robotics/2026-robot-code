package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DrivePIDTestCommand extends Command {
	public DrivePIDTestCommand() {
		addRequirements(drive);
	}

	public double testVelocity = 2;

	@Override
	public void initialize() {
		controls.sysidQuasiForward().onChange(new InstantCommand(() -> {
			testVelocity = -testVelocity;
		}));
	}

	@Override
	public void execute() {
		for (int i = 0; i < drive.modules.length; i++) {
			drive.modules[i].drive(new SwerveModuleState(
					controls.sysidQuasiBackward().getAsBoolean() ? testVelocity : 0, Rotation2d.kZero));
		}
	}
}
