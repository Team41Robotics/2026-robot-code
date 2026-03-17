package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveLock extends Command {
	public static final Rotation2d[] X_ANGLES = {
		Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-45), Rotation2d.fromDegrees(-45), Rotation2d.fromDegrees(45)
	};

	public DriveLock() {
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		drive.setDriveNeutralMode(NeutralModeValue.Brake);
	}

	@Override
	public void execute() {
		for (int i = 0; i < drive.modules.length; i++) {
			drive.modules[i].drive(new SwerveModuleState(0, X_ANGLES[i]));
		}
	}

	@Override
	public void end(boolean interrupted) {
		drive.setDriveNeutralMode(NeutralModeValue.Coast);
	}
}
