package frc.robot.subsystem.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ShooterHW {
	public static final double FLYWHEEL_RATIO = 1.0;

	public static final double FLYWHEEL_kP = 5; // TUNEME
	public static final double FLYWHEEL_kV = 0; //TUNEME
	public static final double FLYWHEEL_kS = 0; //TUNEME

	public TalonFX flywheelTalonFX;
	public TalonFX flywheelFollowerTalonFX;

	public VelocityVoltage flywheelControlRequest = new VelocityVoltage(0).withSlot(0);
	public boolean sysIdFlywheel = false;

	public StatusSignal<AngularVelocity> flywheelVelocity;
	public StatusSignal<Voltage> flywheelMotorVoltage;
	public StatusSignal<Current> flywheelStatorCurrent;
	public StatusSignal<Voltage> flywheelSupplyVoltage;
	public StatusSignal<Current> flywheelSupplyCurrent;
	public StatusSignal<Double> flywheelDutyCycle;
	public StatusSignal<Current> flywheelTorqueCurrent;

	public void init() {
		if (!Robot.isReal()) return;

		// --- Flywheel (leader) ---
		flywheelTalonFX = new TalonFX(51); //TODO 
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
		flywheelFollowerTalonFX = new TalonFX(53); //TODO
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
		flywheelVelocity = flywheelTalonFX.getVelocity(false);
		flywheelMotorVoltage = flywheelTalonFX.getMotorVoltage(false);
		flywheelStatorCurrent = flywheelTalonFX.getStatorCurrent(false);
		flywheelSupplyVoltage = flywheelTalonFX.getSupplyVoltage(false);
		flywheelSupplyCurrent = flywheelTalonFX.getSupplyCurrent(false);
		flywheelDutyCycle = flywheelTalonFX.getDutyCycle(false);
		flywheelTorqueCurrent = flywheelTalonFX.getTorqueCurrent(false);

		// --- Set update frequencies ---
		flywheelVelocity.setUpdateFrequency(50);
		flywheelMotorVoltage.setUpdateFrequency(50);
		flywheelStatorCurrent.setUpdateFrequency(50);
		flywheelSupplyVoltage.setUpdateFrequency(10);
		flywheelSupplyCurrent.setUpdateFrequency(10);
		flywheelDutyCycle.setUpdateFrequency(50);
		flywheelTorqueCurrent.setUpdateFrequency(50);

		// --- Optimize bus utilization ---
		flywheelTalonFX.optimizeBusUtilization();
		flywheelFollowerTalonFX.optimizeBusUtilization();
	}

	public void sense(ShooterInputs inputs) {
		if (!Robot.isReal()) return;

		BaseStatusSignal.refreshAll(
				flywheelVelocity,
				flywheelMotorVoltage,
				flywheelStatorCurrent,
				flywheelSupplyVoltage,
				flywheelSupplyCurrent,
				flywheelDutyCycle,
				flywheelTorqueCurrent);

		inputs.flywheelVelocityRPM = flywheelVelocity.getValueAsDouble() * 60;
		inputs.flywheelVoltageVolts = flywheelMotorVoltage.getValueAsDouble();
		inputs.flywheelCurrentAmps = flywheelStatorCurrent.getValueAsDouble();
		inputs.flywheelBusVoltageVolts = flywheelSupplyVoltage.getValueAsDouble();
		inputs.flywheelBusCurrentAmps = flywheelSupplyCurrent.getValueAsDouble();
	}

	public void actuate(ShooterInputs inputs, double flywheelRPM) {
		Logger.recordOutput("/Shooter/flywheelErrorRPM", inputs.flywheelVelocityRPM - flywheelRPM);
		Logger.recordOutput("/Shooter/currentFlywheelRPM", inputs.flywheelVelocityRPM);

		if (!Robot.isReal()) return;

		if (!sysIdFlywheel) flywheelTalonFX.setControl(flywheelControlRequest.withVelocity(flywheelRPM / 60.0));
	}
}
