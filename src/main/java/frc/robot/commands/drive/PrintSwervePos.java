package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class PrintSwervePos extends Command {
	@Override
	public void execute() {
		for (int i = 0; i < drive.modules.length; i++) {
			System.out.println(drive.modules[i].name + " :  " + drive.modules[i].inputs.turnAbsPosRadians);
		}
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}
