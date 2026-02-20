package frc.robot.subsystem.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Robot;

public class IndexerHW {
	public SparkFlex indexer;

	public void init() {
		if (!Robot.isReal()) return;

		indexer = new SparkFlex(25, MotorType.kBrushless);
		SparkFlexConfig config = new SparkFlexConfig();
		config.idleMode(IdleMode.kCoast);
		indexer.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		indexer.clearFaults();
	}

	public void sense(IndexerInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.voltage = indexer.getAppliedOutput() * indexer.getBusVoltage();
		inputs.current = indexer.getOutputCurrent();
	}

	public void setVoltage(double voltage) {
		if (!Robot.isReal()) return;
		indexer.setVoltage(voltage);
	}
}
