package frc.robot.commands.indexer;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class StopIndexer extends Command {
	public StopIndexer() {
		addRequirements(indexer);
	}

	@Override
	public void initialize() {
		indexer.stop();
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
