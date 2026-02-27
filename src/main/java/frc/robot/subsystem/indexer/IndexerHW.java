package frc.robot.subsystem.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Robot;

public class IndexerHW {
	public TalonFX spinTalonFX;
	public TalonFX elevatorTalonFX;

	public VoltageOut spinControlRequest = new VoltageOut(0);
	public VoltageOut elevatorControlRequest = new VoltageOut(0);

	public void init() {
		if (!Robot.isReal()) return;

		spinTalonFX = new TalonFX(50); // TUNEME
		TalonFXConfiguration spinConfig = new TalonFXConfiguration();
		spinConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		spinConfig.CurrentLimits.SupplyCurrentLimit = 60; // TUNEME
		spinConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TUNEME

		spinTalonFX.getConfigurator().apply(spinConfig);
		spinTalonFX.clearStickyFaults();
		spinTalonFX.setNeutralMode(NeutralModeValue.Brake);

		elevatorTalonFX = new TalonFX(51); // TUNEME
		TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
		elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		elevatorConfig.CurrentLimits.SupplyCurrentLimit = 60; // TUNEME
		elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TUNEME

		elevatorTalonFX.getConfigurator().apply(elevatorConfig);
		elevatorTalonFX.clearStickyFaults();
		elevatorTalonFX.setNeutralMode(NeutralModeValue.Brake);
	}

	public void sense(IndexerInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.spinVoltageVolts = spinTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.spinCurrentAmps = spinTalonFX.getStatorCurrent().getValueAsDouble();
		inputs.spinBusVoltageVolts = spinTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.spinBusCurrentAmps = spinTalonFX.getSupplyCurrent().getValueAsDouble();

		inputs.elevatorVoltageVolts = elevatorTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.elevatorCurrentAmps = elevatorTalonFX.getStatorCurrent().getValueAsDouble();
		inputs.elevatorBusVoltageVolts = elevatorTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.elevatorBusCurrentAmps = elevatorTalonFX.getSupplyCurrent().getValueAsDouble();
	}

	public void actuate(IndexerInputs inputs, double spinVoltage, double elevatorVoltage) {
		if (!Robot.isReal()) return;

		spinTalonFX.setControl(spinControlRequest.withOutput(spinVoltage));
		elevatorTalonFX.setControl(elevatorControlRequest.withOutput(elevatorVoltage));
	}
}
