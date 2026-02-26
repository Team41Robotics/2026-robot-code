package frc.robot.subsystem.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class IndexerHW {
	public TalonFX spinTalonFX;
	public TalonFX elevatorTalonFX;

	public VoltageOut spinControlRequest = new VoltageOut(0);
	public VoltageOut elevatorControlRequest = new VoltageOut(0);

	public void init() {
		if (!Robot.isReal()) return;

		spinTalonFX = new TalonFX(50); // FIXME
		TalonFXConfiguration spinConfig = new TalonFXConfiguration();
		spinConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		spinConfig.CurrentLimits.SupplyCurrentLimit = 60; // FIXME
		spinConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // FIXME

		spinTalonFX.getConfigurator().apply(spinConfig);
		spinTalonFX.clearStickyFaults();
		spinTalonFX.setNeutralMode(NeutralModeValue.Brake);

		elevatorTalonFX = new TalonFX(51); // FIXME
		TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
		elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		elevatorConfig.CurrentLimits.SupplyCurrentLimit = 60; // FIXME
		elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // FIXME

		elevatorTalonFX.getConfigurator().apply(elevatorConfig);
		elevatorTalonFX.clearStickyFaults();
		elevatorTalonFX.setNeutralMode(NeutralModeValue.Brake);
	}

	public void sense(IndexerInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.spinVoltage = spinTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.spinCurrent = spinTalonFX.getStatorCurrent().getValueAsDouble();
		inputs.spinBusVoltage = spinTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.spinBusCurrent = spinTalonFX.getSupplyCurrent().getValueAsDouble();

		inputs.elevatorVoltage = elevatorTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.elevatorCurrent = elevatorTalonFX.getStatorCurrent().getValueAsDouble();
		inputs.elevatorBusVoltage = elevatorTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.elevatorBusCurrent = elevatorTalonFX.getSupplyCurrent().getValueAsDouble();
	}

	public void actuate(IndexerInputs inputs, double spinVoltage, double elevatorVoltage) {
		Logger.recordOutput("/Indexer/actuatedSpinVoltage", spinVoltage);
		Logger.recordOutput("/Indexer/actuatedElevatorVoltage", elevatorVoltage);

		if (!Robot.isReal()) return;

		spinTalonFX.setControl(spinControlRequest.withOutput(spinVoltage));
		elevatorTalonFX.setControl(elevatorControlRequest.withOutput(elevatorVoltage));
	}
}
