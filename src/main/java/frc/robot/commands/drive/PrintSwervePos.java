package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class PrintSwervePos extends Command {

	public PrintSwervePos() {
		addRequirements(drive.subsystem);
	}

	@Override
	public void execute() {
		for (int i = 0; i < drive.modules.length; i++) {
			System.out.println(drive.modules[i].name);
			System.out.println(drive.modules[i].inputs.turnAbsPos);
		}
	}
}
