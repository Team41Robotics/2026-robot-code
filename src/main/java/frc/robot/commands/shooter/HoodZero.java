package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

@SuppressWarnings("static-access")
public class HoodZero extends Command {

	public HoodZero() {
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		shooter.targetHoodPos = 0;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
