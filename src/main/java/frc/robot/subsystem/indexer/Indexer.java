package frc.robot.subsystem.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
	public IndexerHW hw = new IndexerHW();
	public IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

	public double targetRollersVoltage = 0;
	public double targetElevatorVoltage = 0;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Indexer", inputs);
	}

	public void actuate() {
		Logger.recordOutput("/Indexer/targetRollersVoltage", targetRollersVoltage);
		Logger.recordOutput("/Indexer/targetElevatorVoltage", targetElevatorVoltage);

		hw.actuate(inputs, targetRollersVoltage, targetElevatorVoltage);
	}
}
