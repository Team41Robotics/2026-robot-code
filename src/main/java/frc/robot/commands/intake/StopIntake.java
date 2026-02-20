package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class StopIntake extends Command {
	public StopIntake() {
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		intake.stop();
	}

	@Override
	public void execute() {}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {}
}
