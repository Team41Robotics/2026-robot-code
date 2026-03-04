package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class NoDrive extends Command {
	public NoDrive() {
		addRequirements(drive);
	}

	@Override
	public void execute() {
		drive.drive(new ChassisSpeeds());
	}
}
