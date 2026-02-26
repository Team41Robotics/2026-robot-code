package frc.robot.commands.indexer;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class StopIndexer extends Command {
	public StopIndexer() {
		addRequirements(indexer);
	}

	@Override
	public void initialize() {
		indexer.targetSpinVoltage = 0;
		indexer.targetElevatorVoltage = 0;
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
