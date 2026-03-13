package frc.robot.commands.indexer;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class RunIndexer extends Command {
	public static final double DEFAULT_SPIN_VOLTAGE = 2.0; // TUNEME
	public static final double DEFAULT_ELEVATOR_VOLTAGE = 8.0; // TUNEME
	public static final double DEFAULT_BACKVATOR_VOLTAGE = 8.0; // TUNEME

	public double spinVoltage;
	public double elevatorVoltage;
	public double backvatorVoltage;

	public RunIndexer() {
		this(DEFAULT_SPIN_VOLTAGE, DEFAULT_ELEVATOR_VOLTAGE, DEFAULT_BACKVATOR_VOLTAGE);
	}

	public RunIndexer(double spinVoltage, double elevatorVoltage, double backvatorVoltage) {
		this.spinVoltage = spinVoltage;
		this.elevatorVoltage = elevatorVoltage;
		this.backvatorVoltage = backvatorVoltage;
		addRequirements(indexer);
	}

	@Override
	public void initialize() {
		// FIXME shoot only when on target?
		indexer.targetSpinVoltage = spinVoltage;
		indexer.targetElevatorVoltage = elevatorVoltage;
		indexer.targetBackvatorVoltage = backvatorVoltage;
	}

	@Override
	public void end(boolean interrupted) {
		indexer.targetSpinVoltage = 0;
		indexer.targetElevatorVoltage = 0;
		indexer.targetBackvatorVoltage = 0;
	}
}
