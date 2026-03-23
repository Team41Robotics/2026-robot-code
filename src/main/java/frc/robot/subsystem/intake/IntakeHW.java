package frc.robot.subsystem.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class IntakeHW {
	public static final double PINION_RADIUS_METERS = Units.inchesToMeters(1.0); // TODO}
	public static final double GEAR_RATIO = 1.6;

	public static final double EXTENSION_ZERO = 0.0; // TODO

	public static final double EXTENSION_kP = 5; // TUNEME
	public static final double EXTENSION_kI = 0; // TUNEME
	public static final double EXTENSION_kD = 0; // TUNEME

	public TalonFX extensionTalonFX;
	public TalonFX intakeTalonFX;

	public StatusSignal<Angle> extensionAbsolutePosition;
	public StatusSignal<AngularVelocity> extensionVelocity;
	public StatusSignal<Voltage> extensionMotorVoltage;
	public StatusSignal<Current> extensionStatorCurrent;
	public StatusSignal<Voltage> extensionSupplyVoltage;
	public StatusSignal<Current> extensionSupplyCurrent;

	public StatusSignal<AngularVelocity> intakeVelocity;
	public StatusSignal<Voltage> intakeMotorVoltage;
	public StatusSignal<Current> intakeStatorCurrent;
	public StatusSignal<Voltage> intakeSupplyVoltage;
	public StatusSignal<Current> intakeSupplyCurrent;

	public PositionVoltage extensionControlRequest = new PositionVoltage(0).withSlot(0);
	public VoltageOut intakeControlRequest = new VoltageOut(0);

	public void init() {
		if (!Robot.isReal()) return;

		extensionTalonFX = new TalonFX(33); // TODO
		TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
		extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		extensionConfig.Slot0.kP = EXTENSION_kP;
		extensionConfig.Slot0.kI = EXTENSION_kI;
		extensionConfig.Slot0.kD = EXTENSION_kD;
		extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		extensionConfig.CurrentLimits.SupplyCurrentLimit = 30;
		extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		extensionConfig.CurrentLimits.StatorCurrentLimit = 120;
		extensionConfig.Voltage.PeakForwardVoltage = 12.0;
		extensionConfig.Voltage.PeakReverseVoltage = -12.0;
		extensionTalonFX.getConfigurator().apply(extensionConfig);
		extensionTalonFX.clearStickyFaults();
		extensionTalonFX.setNeutralMode(NeutralModeValue.Brake);
		extensionTalonFX.optimizeBusUtilization();

		intakeTalonFX = new TalonFX(31); // TODO
		TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
		intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.SupplyCurrentLimit = 30;
		intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.StatorCurrentLimit = 120;
		intakeConfig.Voltage.PeakForwardVoltage = 12.0;
		intakeConfig.Voltage.PeakReverseVoltage = -12.0;
		intakeTalonFX.getConfigurator().apply(intakeConfig);
		intakeTalonFX.clearStickyFaults();
		intakeTalonFX.setNeutralMode(NeutralModeValue.Coast);
		intakeTalonFX.optimizeBusUtilization();

		extensionAbsolutePosition = extensionTalonFX.getPosition(false);
		extensionVelocity = extensionTalonFX.getVelocity(false);
		extensionMotorVoltage = extensionTalonFX.getMotorVoltage(false);
		extensionStatorCurrent = extensionTalonFX.getStatorCurrent(false);
		extensionSupplyVoltage = extensionTalonFX.getSupplyVoltage(false);
		extensionSupplyCurrent = extensionTalonFX.getSupplyCurrent(false);

		intakeVelocity = intakeTalonFX.getVelocity(false);
		intakeMotorVoltage = intakeTalonFX.getMotorVoltage(false);
		intakeStatorCurrent = intakeTalonFX.getStatorCurrent(false);
		intakeSupplyVoltage = intakeTalonFX.getSupplyVoltage(false);
		intakeSupplyCurrent = intakeTalonFX.getSupplyCurrent(false);

		extensionAbsolutePosition.setUpdateFrequency(50);
		extensionVelocity.setUpdateFrequency(50);
		extensionMotorVoltage.setUpdateFrequency(50);
		extensionStatorCurrent.setUpdateFrequency(50);
		extensionSupplyVoltage.setUpdateFrequency(10);
		extensionSupplyCurrent.setUpdateFrequency(10);

		intakeVelocity.setUpdateFrequency(50);
		intakeMotorVoltage.setUpdateFrequency(50);
		intakeStatorCurrent.setUpdateFrequency(50);
		intakeSupplyVoltage.setUpdateFrequency(10);
		intakeSupplyCurrent.setUpdateFrequency(10);
	}

	public void sense(IntakeInputs inputs) {
		if (!Robot.isReal()) return;

		BaseStatusSignal.refreshAll(
				extensionAbsolutePosition,
				extensionVelocity,
				extensionMotorVoltage,
				extensionStatorCurrent,
				extensionSupplyVoltage,
				extensionSupplyCurrent,
				intakeVelocity,
				intakeMotorVoltage,
				intakeStatorCurrent,
				intakeSupplyVoltage,
				intakeSupplyCurrent);

		inputs.extensionPosMeters =
				extensionAbsolutePosition.getValueAsDouble()* GEAR_RATIO * 2 * Math.PI * PINION_RADIUS_METERS - EXTENSION_ZERO;
		inputs.extensionVelMetersPerSec = extensionVelocity.getValueAsDouble() * GEAR_RATIO * 2 * Math.PI * PINION_RADIUS_METERS;
		inputs.extensionVoltageVolts = extensionMotorVoltage.getValueAsDouble();
		inputs.extensionCurrentAmps = extensionStatorCurrent.getValueAsDouble();
		inputs.extensionBusVoltageVolts = extensionSupplyVoltage.getValueAsDouble();
		inputs.extensionBusCurrentAmps = extensionSupplyCurrent.getValueAsDouble();

		inputs.intakeVelocityRPM = intakeVelocity.getValueAsDouble() * 60;
		inputs.intakeVoltageVolts = intakeMotorVoltage.getValueAsDouble();
		inputs.intakeCurrentAmps = intakeStatorCurrent.getValueAsDouble();
		inputs.intakeBusVoltageVolts = intakeSupplyVoltage.getValueAsDouble();
		inputs.intakeBusCurrentAmps = intakeSupplyCurrent.getValueAsDouble();
	}

	public void actuate(IntakeInputs inputs, double extensionPosition, double intakeVoltage) {
		Logger.recordOutput("/Intake/extensionTarget", extensionPosition);
		Logger.recordOutput("/Intake/intakeVoltageVolts", intakeVoltage);

		if (!Robot.isReal()) return;

		double extensionPositionRotations = (extensionPosition + EXTENSION_ZERO) / (2 * Math.PI * PINION_RADIUS_METERS);

		extensionTalonFX.setControl(extensionControlRequest.withPosition(extensionPositionRotations));
		intakeTalonFX.setControl(intakeControlRequest.withOutput(intakeVoltage));
	}
}
