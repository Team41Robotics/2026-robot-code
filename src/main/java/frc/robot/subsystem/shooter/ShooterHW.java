package frc.robot.subsystem.shooter;

import static java.lang.Math.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ShooterHW {
	public static final double TURRET_RATIO = 1.0 / 16.8571667;
	public static final double HOOD_RATIO = 1.0 / 3.0 / 17.5;
	public static final double FLYWHEEL_RATIO = 1.0;

	// public static final double TURRET_kP = 128.0;
	// public static final double TURRET_kI = 36.0;
	// public static final double TURRET_kD = 0.1;
	public static final double TURRET_kP = 40;
	public static final double TURRET_kI = 0;
	public static final double TURRET_kD = 0;

	public static final double HOOD_kP = 192.0;
	public static final double HOOD_kI = 96.0;
	public static final double HOOD_kD = 0.0;

	public static final double FLYWHEEL_kP = 0.17662;
	public static final double FLYWHEEL_kD = 0;
	public static final double FLYWHEEL_kV = 0.11494;
	public static final double FLYWHEEL_kS = 0.24333;

	public static final double TURRET_NOMINAL_POS = -PI / 2;

	public TalonFX turretTalonFX;
	public TalonFX hoodTalonFX;
	public TalonFX flywheelTalonFX;
	public TalonFX flywheelFollowerTalonFX;

	public PositionVoltage turretControlRequest = new PositionVoltage(0).withSlot(0);
	public PositionVoltage hoodControlRequest = new PositionVoltage(0).withSlot(0);
	public VelocityVoltage flywheelControlRequest = new VelocityVoltage(0).withSlot(0);
	public boolean sysIdFlywheel = false;

	// Cached StatusSignals — turret
	public StatusSignal<Angle> turretPosition;
	public StatusSignal<AngularVelocity> turretVelocity;
	public StatusSignal<Voltage> turretMotorVoltage;
	public StatusSignal<Current> turretStatorCurrent;
	public StatusSignal<Voltage> turretSupplyVoltage;
	public StatusSignal<Current> turretSupplyCurrent;

	// Cached StatusSignals — hood
	public StatusSignal<Angle> hoodPosition;
	public StatusSignal<AngularVelocity> hoodVelocity;
	public StatusSignal<Voltage> hoodMotorVoltage;
	public StatusSignal<Current> hoodStatorCurrent;
	public StatusSignal<Voltage> hoodSupplyVoltage;
	public StatusSignal<Current> hoodSupplyCurrent;

	// Cached StatusSignals — flywheel
	public StatusSignal<AngularVelocity> flywheelVelocity;
	public StatusSignal<Voltage> flywheelMotorVoltage;
	public StatusSignal<Current> flywheelStatorCurrent;
	public StatusSignal<Voltage> flywheelSupplyVoltage;
	public StatusSignal<Current> flywheelSupplyCurrent;
	public StatusSignal<Double> flywheelDutyCycle;
	public StatusSignal<Current> flywheelTorqueCurrent;

	public void init() {
		if (!Robot.isReal()) return;

		// --- Turret ---
		turretTalonFX = new TalonFX(42);
		TalonFXConfiguration turretConfig = new TalonFXConfiguration();
		turretConfig.Feedback.SensorToMechanismRatio = 1.0 / (TURRET_RATIO * 2 * PI);
		turretConfig.Slot0.kP = TURRET_kP;
		turretConfig.Slot0.kI = TURRET_kI;
		turretConfig.Slot0.kD = TURRET_kD;
		turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		turretConfig.CurrentLimits.SupplyCurrentLimit = 20;
		turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		turretConfig.CurrentLimits.StatorCurrentLimit = 30;
		turretConfig.Voltage.PeakForwardVoltage = 12.0;
		turretConfig.Voltage.PeakReverseVoltage = -12.0;
		turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		turretTalonFX.getConfigurator().apply(turretConfig);
		turretTalonFX.clearStickyFaults();
		turretTalonFX.setNeutralMode(NeutralModeValue.Brake);

		turretTalonFX.setPosition(TURRET_NOMINAL_POS);

		// --- Hood ---
		hoodTalonFX = new TalonFX(52);
		TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
		hoodConfig.Feedback.SensorToMechanismRatio = 1.0 / (HOOD_RATIO * 2 * PI);
		hoodConfig.Slot0.kP = HOOD_kP;
		hoodConfig.Slot0.kI = HOOD_kI;
		hoodConfig.Slot0.kD = HOOD_kD;
		hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		hoodConfig.CurrentLimits.SupplyCurrentLimit = 10;
		hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		hoodConfig.CurrentLimits.StatorCurrentLimit = 30;
		hoodConfig.Voltage.PeakForwardVoltage = 12.0;
		hoodConfig.Voltage.PeakReverseVoltage = -12.0;
		hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		hoodTalonFX.getConfigurator().apply(hoodConfig);
		hoodTalonFX.clearStickyFaults();
		hoodTalonFX.setPosition(0);
		hoodTalonFX.setNeutralMode(NeutralModeValue.Brake);

		// --- Flywheel (leader) ---
		flywheelTalonFX = new TalonFX(51);
		TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
		flywheelConfig.Feedback.SensorToMechanismRatio = 1.0 / FLYWHEEL_RATIO;
		flywheelConfig.Slot0.kP = FLYWHEEL_kP;
		flywheelConfig.Slot0.kV = FLYWHEEL_kV;
		flywheelConfig.Slot0.kS = FLYWHEEL_kS;
		flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		flywheelConfig.CurrentLimits.SupplyCurrentLimit = 30;
		flywheelConfig.CurrentLimits.SupplyCurrentLowerTime = 3.0;
		flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		flywheelConfig.CurrentLimits.StatorCurrentLimit = 60;
		flywheelConfig.Voltage.PeakForwardVoltage = 12.0;
		flywheelConfig.Voltage.PeakReverseVoltage = -12.0;
		// flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		flywheelTalonFX.getConfigurator().apply(flywheelConfig);
		flywheelTalonFX.clearStickyFaults();
		flywheelTalonFX.setNeutralMode(NeutralModeValue.Coast);

		// --- Flywheel follower ---
		flywheelFollowerTalonFX = new TalonFX(53);
		TalonFXConfiguration flywheelFollowerConfig = new TalonFXConfiguration();
		flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimit = 30;
		flywheelFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		flywheelFollowerConfig.CurrentLimits.StatorCurrentLimit = 60;
		flywheelFollowerConfig.Voltage.PeakForwardVoltage = 12.0;
		flywheelFollowerConfig.Voltage.PeakReverseVoltage = -12.0;
		flywheelFollowerTalonFX.getConfigurator().apply(flywheelFollowerConfig);
		flywheelFollowerTalonFX.clearStickyFaults();
		flywheelFollowerTalonFX.setNeutralMode(NeutralModeValue.Coast);
		flywheelFollowerTalonFX.setControl(new Follower(51, MotorAlignmentValue.Opposed));

		// --- Cache StatusSignals ---
		turretPosition = turretTalonFX.getPosition(false);
		turretVelocity = turretTalonFX.getVelocity(false);
		turretMotorVoltage = turretTalonFX.getMotorVoltage(false);
		turretStatorCurrent = turretTalonFX.getStatorCurrent(false);
		turretSupplyVoltage = turretTalonFX.getSupplyVoltage(false);
		turretSupplyCurrent = turretTalonFX.getSupplyCurrent(false);

		hoodPosition = hoodTalonFX.getPosition(false);
		hoodVelocity = hoodTalonFX.getVelocity(false);
		hoodMotorVoltage = hoodTalonFX.getMotorVoltage(false);
		hoodStatorCurrent = hoodTalonFX.getStatorCurrent(false);
		hoodSupplyVoltage = hoodTalonFX.getSupplyVoltage(false);
		hoodSupplyCurrent = hoodTalonFX.getSupplyCurrent(false);

		flywheelVelocity = flywheelTalonFX.getVelocity(false);
		flywheelMotorVoltage = flywheelTalonFX.getMotorVoltage(false);
		flywheelStatorCurrent = flywheelTalonFX.getStatorCurrent(false);
		flywheelSupplyVoltage = flywheelTalonFX.getSupplyVoltage(false);
		flywheelSupplyCurrent = flywheelTalonFX.getSupplyCurrent(false);
		flywheelDutyCycle = flywheelTalonFX.getDutyCycle(false);
		flywheelTorqueCurrent = flywheelTalonFX.getTorqueCurrent(false);

		// --- Set update frequencies ---
		turretPosition.setUpdateFrequency(50);
		turretVelocity.setUpdateFrequency(50);
		turretMotorVoltage.setUpdateFrequency(50);
		turretStatorCurrent.setUpdateFrequency(50);
		turretSupplyVoltage.setUpdateFrequency(10);
		turretSupplyCurrent.setUpdateFrequency(10);

		hoodPosition.setUpdateFrequency(50);
		hoodVelocity.setUpdateFrequency(50);
		hoodMotorVoltage.setUpdateFrequency(50);
		hoodStatorCurrent.setUpdateFrequency(50);
		hoodSupplyVoltage.setUpdateFrequency(10);
		hoodSupplyCurrent.setUpdateFrequency(10);

		flywheelVelocity.setUpdateFrequency(50);
		flywheelMotorVoltage.setUpdateFrequency(50);
		flywheelStatorCurrent.setUpdateFrequency(50);
		flywheelSupplyVoltage.setUpdateFrequency(10);
		flywheelSupplyCurrent.setUpdateFrequency(10);
		flywheelDutyCycle.setUpdateFrequency(50);
		flywheelTorqueCurrent.setUpdateFrequency(50);

		// --- Optimize bus utilization ---
		turretTalonFX.optimizeBusUtilization();
		hoodTalonFX.optimizeBusUtilization();
		flywheelTalonFX.optimizeBusUtilization();
		flywheelFollowerTalonFX.optimizeBusUtilization();
	}

	public void sense(ShooterInputs inputs) {
		if (!Robot.isReal()) return;

		BaseStatusSignal.refreshAll(
				turretPosition,
				turretVelocity,
				turretMotorVoltage,
				turretStatorCurrent,
				turretSupplyVoltage,
				turretSupplyCurrent,
				hoodPosition,
				hoodVelocity,
				hoodMotorVoltage,
				hoodStatorCurrent,
				hoodSupplyVoltage,
				hoodSupplyCurrent,
				flywheelVelocity,
				flywheelMotorVoltage,
				flywheelStatorCurrent,
				flywheelSupplyVoltage,
				flywheelSupplyCurrent,
				flywheelDutyCycle,
				flywheelTorqueCurrent);

		inputs.turretPosRadians = turretPosition.getValueAsDouble();

		inputs.turretVelRadiansPerSec = turretVelocity.getValueAsDouble();
		inputs.turretVoltageVolts = turretMotorVoltage.getValueAsDouble();
		inputs.turretCurrentAmps = turretStatorCurrent.getValueAsDouble();
		inputs.turretBusVoltageVolts = turretSupplyVoltage.getValueAsDouble();
		inputs.turretBusCurrentAmps = turretSupplyCurrent.getValueAsDouble();
		inputs.turretTsSec = turretPosition.getTimestamp().getTime();

		inputs.hoodPosRadians = hoodPosition.getValueAsDouble();

		inputs.hoodVelRadiansPerSec = hoodVelocity.getValueAsDouble();
		inputs.hoodVoltageVolts = hoodMotorVoltage.getValueAsDouble();
		inputs.hoodCurrentAmps = hoodStatorCurrent.getValueAsDouble();
		inputs.hoodBusVoltageVolts = hoodSupplyVoltage.getValueAsDouble();
		inputs.hoodBusCurrentAmps = hoodSupplyCurrent.getValueAsDouble();
		inputs.hoodTsSec = hoodPosition.getTimestamp().getTime();

		inputs.flywheelVelocityRPM = flywheelVelocity.getValueAsDouble() * 60;
		inputs.flywheelVoltageVolts = flywheelMotorVoltage.getValueAsDouble();
		inputs.flywheelCurrentAmps = flywheelStatorCurrent.getValueAsDouble();
		inputs.flywheelBusVoltageVolts = flywheelSupplyVoltage.getValueAsDouble();
		inputs.flywheelBusCurrentAmps = flywheelSupplyCurrent.getValueAsDouble();
		inputs.flywheelTsSec = flywheelVelocity.getTimestamp().getTime();
	}

	public void actuate(ShooterInputs inputs, double turretPosition, double hoodPosition, double flywheelRPM) {
		Logger.recordOutput("/Shooter/turretErrorRadians", inputs.turretPosRadians - turretPosition);
		Logger.recordOutput("/Shooter/hoodErrorRadians", inputs.hoodPosRadians - hoodPosition);
		Logger.recordOutput("/Shooter/flywheelErrorRPM", inputs.flywheelVelocityRPM - flywheelRPM);

		if (!Robot.isReal()) return;

		turretTalonFX.setControl(turretControlRequest.withPosition(turretPosition));
		// Always clamp hood position to 0 if it goes negative
		if (inputs.hoodPosRadians < 0) {
			hoodTalonFX.setPosition(0);
			hoodPosition = 0;
		}
		if (hoodPosition < 0.01) {
			hoodTalonFX.setControl(hoodControlRequest.withPosition(hoodPosition).withFeedForward(-0.5));
		} else {
			hoodTalonFX.setControl(hoodControlRequest.withPosition(hoodPosition));
		}
		if (!sysIdFlywheel) flywheelTalonFX.setControl(flywheelControlRequest.withVelocity(flywheelRPM / 60.0));
	}
}
