package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterStartup extends Command {
	public ShooterStartup() {
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		Targetting.loadData();
		shooter.targetFlywheelRPM = 0;
	}

	@Override
	public void end(boolean interrupted) {
		shooter.targetFlywheelRPM = 0;
	}
}
