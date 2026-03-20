package frc.robot.commands.indexer;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class RunIndexer extends Command {
	public static final double DEFAULT_SPIN_VOLTAGE = 2.0;
	public static final double DEFAULT_ELEVATOR_VOLTAGE = 8.0;
	public static final double DEFAULT_BACKVATOR_VOLTAGE = 8.0;

	public double spinVoltage;
	public double elevatorVoltage;
	public double backvatorVoltage;
	public boolean waitForOnTarget;

	public RunIndexer() {
		this(DEFAULT_SPIN_VOLTAGE, DEFAULT_ELEVATOR_VOLTAGE, DEFAULT_BACKVATOR_VOLTAGE, true);
	}

	public RunIndexer(double spinVoltage, double elevatorVoltage, double backvatorVoltage) {
		this(spinVoltage, elevatorVoltage, backvatorVoltage, false);
	}

	public RunIndexer(double spinVoltage, double elevatorVoltage, double backvatorVoltage, boolean waitForOnTarget) {
		this.spinVoltage = spinVoltage;
		this.elevatorVoltage = elevatorVoltage;
		this.backvatorVoltage = backvatorVoltage;
		this.waitForOnTarget = waitForOnTarget;
		addRequirements(indexer);
	}

	@Override
	public void execute() {
		if (!waitForOnTarget || shooter.onTarget) {
			indexer.targetSpinVoltage = spinVoltage;
			indexer.targetElevatorVoltage = elevatorVoltage;
			indexer.targetBackvatorVoltage = backvatorVoltage;
		} else {
			indexer.targetSpinVoltage = 0;
			indexer.targetElevatorVoltage = 0;
			indexer.targetBackvatorVoltage = 0;
		}
	}

	@Override
	public void end(boolean interrupted) {
		indexer.targetSpinVoltage = 0;
		indexer.targetElevatorVoltage = 0;
		indexer.targetBackvatorVoltage = 0;
	}
}
