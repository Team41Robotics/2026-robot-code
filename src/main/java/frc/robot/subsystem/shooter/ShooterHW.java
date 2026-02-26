package frc.robot.subsystem.shooter;

import static java.lang.Math.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class ShooterHW {
	public static final double TURRET_RATIO = 0.05; // FIXME
	public static final double HOOD_RATIO = 1.0; // FIXME
	public static final double FLYWHEEL_RATIO = 1.0; // FIXME

	public static final double TURRET_kP = 5.0; // FIXME
	public static final double TURRET_kD = 0.0; // FIXME

	public static final double HOOD_kP = 0.5; // FIXME
	public static final double HOOD_kD = 0.0; // FIXME

	public static final double FLYWHEEL_kP = 0.1; // FIXME
	public static final double FLYWHEEL_kV = 0.12; // FIXME
	public static final double FLYWHEEL_kS = 0.0; // FIXME

	public TalonFX turretTalonFX;
	public TalonFX hoodTalonFX;
	public TalonFX flywheelTalonFX;
	public TalonFX flywheelFollowerTalonFX;

	public PositionVoltage turretControlRequest = new PositionVoltage(0).withSlot(0);
	public PositionVoltage hoodControlRequest = new PositionVoltage(0).withSlot(0);
	public VelocityVoltage flywheelControlRequest = new VelocityVoltage(0).withSlot(0);

	public DigitalInput hoodLimitSwitch;

	public void init() {
		if (!Robot.isReal()) return;

		// --- Turret ---
		turretTalonFX = new TalonFX(40); // FIXME
		TalonFXConfiguration turretConfig = new TalonFXConfiguration();
		turretConfig.Slot0.kP = TURRET_kP * TURRET_RATIO * 2 * PI;
		turretConfig.Slot0.kD = TURRET_kD * TURRET_RATIO * 2 * PI;
		turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		turretConfig.CurrentLimits.SupplyCurrentLimit = 30; // FIXME
		turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		turretConfig.CurrentLimits.StatorCurrentLimit = 60; // FIXME
		turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // FIXME
		turretTalonFX.getConfigurator().apply(turretConfig);
		turretTalonFX.clearStickyFaults();
		turretTalonFX.setPosition(0);
		turretTalonFX.setNeutralMode(NeutralModeValue.Brake);

		// --- Hood ---
		hoodTalonFX = new TalonFX(41); // FIXME
		hoodLimitSwitch = new DigitalInput(0); // FIXME
		TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
		hoodConfig.Slot0.kP = HOOD_kP * HOOD_RATIO * 2 * PI;
		hoodConfig.Slot0.kD = HOOD_kD * HOOD_RATIO * 2 * PI;
		hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		hoodConfig.CurrentLimits.SupplyCurrentLimit = 30; // FIXME
		hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		hoodConfig.CurrentLimits.StatorCurrentLimit = 40; // FIXME
		hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // FIXME
		hoodTalonFX.getConfigurator().apply(hoodConfig);
		hoodTalonFX.clearStickyFaults();
		hoodTalonFX.setPosition(0);
		hoodTalonFX.setNeutralMode(NeutralModeValue.Brake);

		// --- Flywheel (leader) ---
		flywheelTalonFX = new TalonFX(42); // FIXME
		TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
		flywheelConfig.Slot0.kP = FLYWHEEL_kP * FLYWHEEL_RATIO * 2 * PI;
		flywheelConfig.Slot0.kV = FLYWHEEL_kV * FLYWHEEL_RATIO * 2 * PI;
		flywheelConfig.Slot0.kS = FLYWHEEL_kS;
		flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		flywheelConfig.CurrentLimits.SupplyCurrentLimit = 40; // FIXME
		flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		flywheelConfig.CurrentLimits.StatorCurrentLimit = 80; // FIXME
		flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // FIXME
		flywheelTalonFX.getConfigurator().apply(flywheelConfig);
		flywheelTalonFX.clearStickyFaults();
		flywheelTalonFX.setNeutralMode(NeutralModeValue.Coast);

		// --- Flywheel follower ---
		flywheelFollowerTalonFX = new TalonFX(43); // FIXME
		flywheelFollowerTalonFX.getConfigurator().apply(new TalonFXConfiguration());
		flywheelFollowerTalonFX.clearStickyFaults();
		flywheelFollowerTalonFX.setNeutralMode(NeutralModeValue.Coast);
		flywheelFollowerTalonFX.setControl(new Follower(42, MotorAlignmentValue.Opposed)); // FIXME
	}

	public void sense(ShooterInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.isHoodLimitSwitchOn = hoodLimitSwitch.get();
		if (inputs.isHoodLimitSwitchOn) {
			hoodTalonFX.setPosition(0);
		}

		inputs.turretPos = turretTalonFX.getPosition().getValueAsDouble() * TURRET_RATIO * 2 * PI;
		inputs.turretVel = turretTalonFX.getVelocity().getValueAsDouble() * TURRET_RATIO * 2 * PI;
		inputs.turretVoltage = turretTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.turretCurrent = turretTalonFX.getStatorCurrent().getValueAsDouble();
		inputs.turretBusVoltage = turretTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.turretBusCurrent = turretTalonFX.getSupplyCurrent().getValueAsDouble();

		inputs.hoodPos = hoodTalonFX.getPosition().getValueAsDouble() * HOOD_RATIO * 2 * PI;
		inputs.hoodVel = hoodTalonFX.getVelocity().getValueAsDouble() * HOOD_RATIO * 2 * PI;
		inputs.hoodVoltage = hoodTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.hoodCurrent = hoodTalonFX.getStatorCurrent().getValueAsDouble();
		inputs.hoodBusVoltage = hoodTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.hoodBusCurrent = hoodTalonFX.getSupplyCurrent().getValueAsDouble();

		inputs.flywheelVel = flywheelTalonFX.getVelocity().getValueAsDouble() * FLYWHEEL_RATIO * 2 * PI;
		inputs.flywheelVoltage = flywheelTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.flywheelCurrent = flywheelTalonFX.getStatorCurrent().getValueAsDouble();
		inputs.flywheelBusVoltage = flywheelTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.flywheelBusCurrent = flywheelTalonFX.getSupplyCurrent().getValueAsDouble();
	}

	public void actuate(ShooterInputs inputs, double turretPosition, double hoodPosition, double flywheelVelocity) {
		Logger.recordOutput("/Shooter/turretErrorRad", inputs.turretPos - turretPosition);
		Logger.recordOutput("/Shooter/hoodErrorRad", inputs.hoodPos - hoodPosition);
		Logger.recordOutput("/Shooter/flywheelErrorRadPerSec", inputs.flywheelVel - flywheelVelocity);

		if (!Robot.isReal()) return;

		turretTalonFX.setControl(turretControlRequest.withPosition(turretPosition / (TURRET_RATIO * 2 * PI)));
		hoodTalonFX.setControl(hoodControlRequest.withPosition(hoodPosition / (HOOD_RATIO * 2 * PI)));
		flywheelTalonFX.setControl(flywheelControlRequest.withVelocity(flywheelVelocity / (FLYWHEEL_RATIO * 2 * PI)));
		// follower automatically follows leader
	}
}
