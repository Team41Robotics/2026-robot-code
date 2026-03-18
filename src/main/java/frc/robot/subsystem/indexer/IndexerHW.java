package frc.robot.subsystem.indexer;

import static frc.robot.RobotContainer.*;

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
	public TalonFX rollersTalonFX;
	public TalonFX elevatorTalonFX;

	public VoltageOut rollersControlRequest = new VoltageOut(0);
	public VoltageOut elevatorControlRequest = new VoltageOut(0);

	public StatusSignal<AngularVelocity> rollersVelocity;
	public StatusSignal<Voltage> rollersMotorVoltage;
	public StatusSignal<Current> rollersStatorCurrent;
	public StatusSignal<Voltage> rollersSupplyVoltage;
	public StatusSignal<Current> rollersSupplyCurrent;

	public StatusSignal<AngularVelocity> elevatorVelocity;
	public StatusSignal<Voltage> elevatorMotorVoltage;
	public StatusSignal<Current> elevatorStatorCurrent;
	public StatusSignal<Voltage> elevatorSupplyVoltage;
	public StatusSignal<Current> elevatorSupplyCurrent;

	public void init() {
		if (!Robot.isReal()) return;

		rollersTalonFX = new TalonFX(43, driveBus); //TODO
		TalonFXConfiguration rollersConfig = new TalonFXConfiguration();
		rollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		rollersConfig.CurrentLimits.SupplyCurrentLimit = 30;
		rollersConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		rollersConfig.CurrentLimits.StatorCurrentLimit = 120;
		rollersConfig.Voltage.PeakForwardVoltage = 12.0;
		rollersConfig.Voltage.PeakReverseVoltage = -12.0;
		rollersConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		rollersTalonFX.getConfigurator().apply(rollersConfig);
		rollersTalonFX.clearStickyFaults();
		rollersTalonFX.setNeutralMode(NeutralModeValue.Brake);
		rollersVelocity = rollersTalonFX.getVelocity(false);
		rollersMotorVoltage = rollersTalonFX.getMotorVoltage(false);
		rollersStatorCurrent = rollersTalonFX.getStatorCurrent(false);
		rollersSupplyVoltage = rollersTalonFX.getSupplyVoltage(false);
		rollersSupplyCurrent = rollersTalonFX.getSupplyCurrent(false);

		rollersVelocity.setUpdateFrequency(50);
		rollersMotorVoltage.setUpdateFrequency(50);
		rollersStatorCurrent.setUpdateFrequency(50);
		rollersSupplyVoltage.setUpdateFrequency(50);
		rollersSupplyCurrent.setUpdateFrequency(50);

		rollersTalonFX.optimizeBusUtilization();

		elevatorTalonFX = new TalonFX(41); //TODO
		TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
		elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		elevatorConfig.CurrentLimits.SupplyCurrentLimit = 20;
		elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		elevatorConfig.CurrentLimits.StatorCurrentLimit = 40;
		elevatorConfig.Voltage.PeakForwardVoltage = 12.0;
		elevatorConfig.Voltage.PeakReverseVoltage = -12.0;
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
		elevatorSupplyVoltage.setUpdateFrequency(10);
		elevatorSupplyCurrent.setUpdateFrequency(10);

		elevatorTalonFX.optimizeBusUtilization();

	}

	public void sense(IndexerInputs inputs) {
		if (!Robot.isReal()) return;

		BaseStatusSignal.waitForAll(
				0, rollersVelocity, rollersMotorVoltage, rollersStatorCurrent, rollersSupplyVoltage, rollersSupplyCurrent);
		BaseStatusSignal.refreshAll(
				elevatorVelocity,
				elevatorMotorVoltage,
				elevatorStatorCurrent,
				elevatorSupplyVoltage,
				elevatorSupplyCurrent);

		inputs.rollersVelocityRPM = rollersVelocity.getValueAsDouble() * 60.0;
		inputs.rollersVoltageVolts = rollersMotorVoltage.getValueAsDouble();
		inputs.rollersCurrentAmps = rollersStatorCurrent.getValueAsDouble();
		inputs.rollersBusVoltageVolts = rollersSupplyVoltage.getValueAsDouble();
		inputs.rollersBusCurrentAmps = rollersSupplyCurrent.getValueAsDouble();

		inputs.elevatorVelocityRPM = elevatorVelocity.getValueAsDouble() * 60.0;
		inputs.elevatorVoltageVolts = elevatorMotorVoltage.getValueAsDouble();
		inputs.elevatorCurrentAmps = elevatorStatorCurrent.getValueAsDouble();
		inputs.elevatorBusVoltageVolts = elevatorSupplyVoltage.getValueAsDouble();
		inputs.elevatorBusCurrentAmps = elevatorSupplyCurrent.getValueAsDouble();
	}

	public void actuate(IndexerInputs inputs, double spinVoltage, double elevatorVoltage) {
		if (!Robot.isReal()) return;

		rollersTalonFX.setControl(rollersControlRequest.withOutput(spinVoltage));
		elevatorTalonFX.setControl(elevatorControlRequest.withOutput(elevatorVoltage));
	}
}
