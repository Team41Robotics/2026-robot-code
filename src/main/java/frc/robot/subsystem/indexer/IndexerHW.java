package frc.robot.subsystem.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;

public class IndexerHW {
	public TalonFX spinTalonFX;
	public TalonFX elevatorTalonFX;

	public VoltageOut spinControlRequest = new VoltageOut(0);
	public VoltageOut elevatorControlRequest = new VoltageOut(0);

	public StatusSignal<AngularVelocity> spinVelocity;
	public StatusSignal<Voltage> spinMotorVoltage;
	public StatusSignal<Current> spinStatorCurrent;
	public StatusSignal<Voltage> spinSupplyVoltage;
	public StatusSignal<Current> spinSupplyCurrent;

	public StatusSignal<AngularVelocity> elevatorVelocity;
	public StatusSignal<Voltage> elevatorMotorVoltage;
	public StatusSignal<Current> elevatorStatorCurrent;
	public StatusSignal<Voltage> elevatorSupplyVoltage;
	public StatusSignal<Current> elevatorSupplyCurrent;

	public void init() {
		if (!Robot.isReal()) return;

		spinTalonFX = new TalonFX(43);
		TalonFXConfiguration spinConfig = new TalonFXConfiguration();
		spinConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		spinConfig.CurrentLimits.SupplyCurrentLimit = 60;
		spinConfig.CurrentLimits.StatorCurrentLimitEnable = false;
		spinConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		spinTalonFX.getConfigurator().apply(spinConfig);
		spinTalonFX.clearStickyFaults();
		spinTalonFX.setNeutralMode(NeutralModeValue.Brake);

		spinVelocity = spinTalonFX.getVelocity(false);
		spinMotorVoltage = spinTalonFX.getMotorVoltage(false);
		spinStatorCurrent = spinTalonFX.getStatorCurrent(false);
		spinSupplyVoltage = spinTalonFX.getSupplyVoltage(false);
		spinSupplyCurrent = spinTalonFX.getSupplyCurrent(false);

		spinVelocity.setUpdateFrequency(50);
		spinMotorVoltage.setUpdateFrequency(50);
		spinStatorCurrent.setUpdateFrequency(50);
		spinSupplyVoltage.setUpdateFrequency(50);
		spinSupplyCurrent.setUpdateFrequency(50);

		spinTalonFX.optimizeBusUtilization();

		elevatorTalonFX = new TalonFX(41);
		TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
		elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		elevatorConfig.CurrentLimits.SupplyCurrentLimit = 60;
		elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		elevatorTalonFX.getConfigurator().apply(elevatorConfig);
		elevatorTalonFX.clearStickyFaults();
		elevatorTalonFX.setNeutralMode(NeutralModeValue.Brake);

		elevatorVelocity = elevatorTalonFX.getVelocity(false);
		elevatorMotorVoltage = elevatorTalonFX.getMotorVoltage(false);
		elevatorStatorCurrent = elevatorTalonFX.getStatorCurrent(false);
		elevatorSupplyVoltage = elevatorTalonFX.getSupplyVoltage(false);
		elevatorSupplyCurrent = elevatorTalonFX.getSupplyCurrent(false);

		elevatorVelocity.setUpdateFrequency(50);
		elevatorMotorVoltage.setUpdateFrequency(50);
		elevatorStatorCurrent.setUpdateFrequency(50);
		elevatorSupplyVoltage.setUpdateFrequency(50);
		elevatorSupplyCurrent.setUpdateFrequency(50);

		elevatorTalonFX.optimizeBusUtilization();
	}

	public void sense(IndexerInputs inputs) {
		if (!Robot.isReal()) return;

		BaseStatusSignal.refreshAll(
				spinVelocity,
				spinMotorVoltage,
				spinStatorCurrent,
				spinSupplyVoltage,
				spinSupplyCurrent,
				elevatorVelocity,
				elevatorMotorVoltage,
				elevatorStatorCurrent,
				elevatorSupplyVoltage,
				elevatorSupplyCurrent);

		inputs.spinVelocityRPM = spinVelocity.getValueAsDouble() * 60.0;
		inputs.spinVoltageVolts = spinMotorVoltage.getValueAsDouble();
		inputs.spinCurrentAmps = spinStatorCurrent.getValueAsDouble();
		inputs.spinBusVoltageVolts = spinSupplyVoltage.getValueAsDouble();
		inputs.spinBusCurrentAmps = spinSupplyCurrent.getValueAsDouble();

		inputs.elevatorVelocityRPM = elevatorVelocity.getValueAsDouble() * 60.0;
		inputs.elevatorVoltageVolts = elevatorMotorVoltage.getValueAsDouble();
		inputs.elevatorCurrentAmps = elevatorStatorCurrent.getValueAsDouble();
		inputs.elevatorBusVoltageVolts = elevatorSupplyVoltage.getValueAsDouble();
		inputs.elevatorBusCurrentAmps = elevatorSupplyCurrent.getValueAsDouble();
	}

	public void actuate(IndexerInputs inputs, double spinVoltage, double elevatorVoltage) {
		if (!Robot.isReal()) return;

		// spinTalonFX.setControl(spinControlRequest.withOutput(spinVoltage));
		elevatorTalonFX.setControl(elevatorControlRequest.withOutput(elevatorVoltage));
	}
}
