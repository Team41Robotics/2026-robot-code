package frc.robot.commands.indexer;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class RunIndexer extends Command {
	public static final double DEFAULT_ROLLERS_VOLTAGE = 2.0; // TUNEME
	public static final double DEFAULT_ELEVATOR_VOLTAGE = 8.0; // TUNEME

	public double rollersVoltage;
	public double elevatorVoltage;
	
	public RunIndexer() {
		this(DEFAULT_ROLLERS_VOLTAGE, DEFAULT_ELEVATOR_VOLTAGE);
	}

	public RunIndexer(double rollersVoltage, double elevatorVoltage) {
		this.rollersVoltage = rollersVoltage;
		this.elevatorVoltage = elevatorVoltage;
		addRequirements(indexer);
	}

	@Override
	public void initialize() {
		// FIXME shoot only when on target?
		indexer.targetRollersVoltage = rollersVoltage;
		indexer.targetElevatorVoltage = elevatorVoltage;
	}

	@Override
	public void end(boolean interrupted) {
		indexer.targetRollersVoltage = 0;
		indexer.targetElevatorVoltage = 0;
	}
}
