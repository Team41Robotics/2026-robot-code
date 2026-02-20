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
		config.smartCurrentLimit(40);
		indexer.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		indexer.clearFaults();
	}

	public void sense(IndexerInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.velocityRPM = indexer.getEncoder().getVelocity();
		inputs.busVoltageVolts = indexer.getBusVoltage();
		inputs.voltageVolts = indexer.getAppliedOutput() * inputs.busVoltageVolts;
		inputs.currentAmps = indexer.getOutputCurrent();
	}

	public void setVoltage(double voltage) {
		if (!Robot.isReal()) return;
		indexer.setVoltage(voltage);
	}
}
