package frc.robot.commands.indexer;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class StartIndexer extends Command {
	public double voltage;

	public StartIndexer() {
		this(2.0);
	}

	public StartIndexer(double voltage) {
		this.voltage = voltage;
		addRequirements(indexer);
	}

	@Override
	public void initialize() {
		indexer.setVoltage(voltage);
	}

	@Override
	public void execute() {}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		indexer.stop();
	}
}
