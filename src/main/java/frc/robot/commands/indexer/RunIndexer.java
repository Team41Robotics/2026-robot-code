package frc.robot.commands.indexer;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class RunIndexer extends Command {
	public static final double DEFAULT_SPIN_VOLTAGE = 8.0; // TUNEME
	public static final double DEFAULT_ELEVATOR_VOLTAGE = 8.0; // TUNEME

	public double spinVoltage;
	public double elevatorVoltage;

	public RunIndexer() {
		this(DEFAULT_SPIN_VOLTAGE, DEFAULT_ELEVATOR_VOLTAGE);
	}

	public RunIndexer(double spinVoltage, double elevatorVoltage) {
		this.spinVoltage = spinVoltage;
		this.elevatorVoltage = elevatorVoltage;
		addRequirements(indexer);
	}

	@Override
	public void initialize() {
		indexer.targetSpinVoltage = spinVoltage;
		indexer.targetElevatorVoltage = elevatorVoltage;
	}

	@Override
	public void end(boolean interrupted) {
		indexer.targetSpinVoltage = 0;
		indexer.targetElevatorVoltage = 0;
	}
}
