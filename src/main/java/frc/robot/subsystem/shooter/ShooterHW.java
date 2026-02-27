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
	public static final double TURRET_RATIO = 0.05; // TUNEME
	public static final double HOOD_RATIO = 1.0; // TUNEME
	public static final double FLYWHEEL_RATIO = 1.0; // TUNEME

	public static final double TURRET_kP = 5.0; // TUNEME
	public static final double TURRET_kD = 0.0; // TUNEME

	public static final double HOOD_kP = 0.5; // TUNEME
	public static final double HOOD_kD = 0.0; // TUNEME

	public static final double FLYWHEEL_kP = 0.1; // TUNEME
	public static final double FLYWHEEL_kV = 0.12; // TUNEME
	public static final double FLYWHEEL_kS = 0.0; // TUNEME

	public static final double ENCODER_LIM_POS1LEFT = -2.0; // TUNEME. turret left limit (rad)
	public static final double ENCODER_LIM_POS1RIGHT = 2.0; // TUNEME. turret right limit (rad)
	public static final double ENCODER_LIM_POS2LEFT = -1.0; // TUNEME. hood left limit (rad)
	public static final double ENCODER_LIM_POS2RIGHT = 1.0; // TUNEME. hood right limit (rad)

	public TalonFX turretTalonFX;
	public TalonFX hoodTalonFX;
	public TalonFX flywheelTalonFX;
	public TalonFX flywheelFollowerTalonFX;

	public PositionVoltage turretControlRequest = new PositionVoltage(0).withSlot(0);
	public PositionVoltage hoodControlRequest = new PositionVoltage(0).withSlot(0);
	public VelocityVoltage flywheelControlRequest = new VelocityVoltage(0).withSlot(0);

	public DigitalInput hoodLimitSwitch;
	public DigitalInput turretLimitSwitch;

	public void init() {
		if (!Robot.isReal()) return;

		// --- Turret ---
		turretTalonFX = new TalonFX(40); // HACK
		TalonFXConfiguration turretConfig = new TalonFXConfiguration();
		turretConfig.Feedback.SensorToMechanismRatio = 1.0 / (TURRET_RATIO * 2 * PI);
		turretConfig.Slot0.kP = TURRET_kP;
		turretConfig.Slot0.kD = TURRET_kD;
		turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		turretConfig.CurrentLimits.SupplyCurrentLimit = 30; // TUNEME
		turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		turretConfig.CurrentLimits.StatorCurrentLimit = 60; // TUNEME
		turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TUNEME
		turretTalonFX.getConfigurator().apply(turretConfig);
		turretTalonFX.clearStickyFaults();
		turretTalonFX.setPosition(0);
		turretTalonFX.setNeutralMode(NeutralModeValue.Brake);

		turretLimitSwitch = new DigitalInput(1); // HACK

		// --- Hood ---
		hoodTalonFX = new TalonFX(41); // HACK
		TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
		hoodConfig.Feedback.SensorToMechanismRatio = 1.0 / (HOOD_RATIO * 2 * PI);
		hoodConfig.Slot0.kP = HOOD_kP;
		hoodConfig.Slot0.kD = HOOD_kD;
		hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		hoodConfig.CurrentLimits.SupplyCurrentLimit = 30; // TUNEME
		hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		hoodConfig.CurrentLimits.StatorCurrentLimit = 40; // TUNEME
		hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TUNEME
		hoodTalonFX.getConfigurator().apply(hoodConfig);
		hoodTalonFX.clearStickyFaults();
		hoodTalonFX.setPosition(0);
		hoodTalonFX.setNeutralMode(NeutralModeValue.Brake);

		hoodLimitSwitch = new DigitalInput(0); // HACK

		// --- Flywheel (leader) ---
		flywheelTalonFX = new TalonFX(42); // HACK
		TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
		flywheelConfig.Feedback.SensorToMechanismRatio = 1.0 / (FLYWHEEL_RATIO / 60.0); // mechanism unit = RPM
		flywheelConfig.Slot0.kP = FLYWHEEL_kP;
		flywheelConfig.Slot0.kV = FLYWHEEL_kV;
		flywheelConfig.Slot0.kS = FLYWHEEL_kS;
		flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		flywheelConfig.CurrentLimits.SupplyCurrentLimit = 40; // TUNEME
		flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		flywheelConfig.CurrentLimits.StatorCurrentLimit = 80; // TUNEME
		flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TUNEME
		flywheelTalonFX.getConfigurator().apply(flywheelConfig);
		flywheelTalonFX.clearStickyFaults();
		flywheelTalonFX.setNeutralMode(NeutralModeValue.Coast);

		// --- Flywheel follower ---
		flywheelFollowerTalonFX = new TalonFX(43); // HACK
		flywheelFollowerTalonFX.getConfigurator().apply(new TalonFXConfiguration());
		flywheelFollowerTalonFX.clearStickyFaults();
		flywheelFollowerTalonFX.setNeutralMode(NeutralModeValue.Coast);
		flywheelFollowerTalonFX.setControl(new Follower(42, MotorAlignmentValue.Opposed)); // HACK
	}

	public void sense(ShooterInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.turretLimitSwitchOn = turretLimitSwitch.get();
		inputs.turretPosRadians = turretTalonFX.getPosition().getValueAsDouble();
		if (inputs.turretLimitSwitchOn) {
			double bestPos = Double.POSITIVE_INFINITY;
			if (abs(inputs.turretPosRadians - ENCODER_LIM_POS1LEFT) < abs(inputs.turretPosRadians - bestPos)) {
				bestPos = ENCODER_LIM_POS1LEFT;
			}
			if (abs(inputs.turretPosRadians - ENCODER_LIM_POS1RIGHT) < abs(inputs.turretPosRadians - bestPos)) {
				bestPos = ENCODER_LIM_POS1RIGHT;
			}
			if (ENCODER_LIM_POS1LEFT < inputs.turretPosRadians && inputs.turretPosRadians < ENCODER_LIM_POS1RIGHT) {
				bestPos = inputs.turretPosRadians;
			}
			if (abs(inputs.turretPosRadians - ENCODER_LIM_POS2LEFT) < abs(inputs.turretPosRadians - bestPos)) {
				bestPos = ENCODER_LIM_POS2LEFT;
			}
			if (abs(inputs.turretPosRadians - ENCODER_LIM_POS2RIGHT) < abs(inputs.turretPosRadians - bestPos)) {
				bestPos = ENCODER_LIM_POS2RIGHT;
			}
			if (ENCODER_LIM_POS2LEFT < inputs.turretPosRadians && inputs.turretPosRadians < ENCODER_LIM_POS2RIGHT) {
				bestPos = inputs.turretPosRadians;
			}
			turretTalonFX.setPosition(bestPos);
			inputs.turretPosRadians = bestPos;
		}

		inputs.turretVelRadiansPerSec = turretTalonFX.getVelocity().getValueAsDouble();
		inputs.turretVoltageVolts = turretTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.turretCurrentAmps = turretTalonFX.getStatorCurrent().getValueAsDouble();
		inputs.turretBusVoltageVolts = turretTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.turretBusCurrentAmps = turretTalonFX.getSupplyCurrent().getValueAsDouble();

		inputs.isHoodLimitSwitchOn = hoodLimitSwitch.get();
		inputs.hoodPosRadians = hoodTalonFX.getPosition().getValueAsDouble();
		if (inputs.isHoodLimitSwitchOn) {
			hoodTalonFX.setPosition(0);
			inputs.hoodPosRadians = 0;
		}

		inputs.hoodVelRadiansPerSec = hoodTalonFX.getVelocity().getValueAsDouble();
		inputs.hoodVoltageVolts = hoodTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.hoodCurrentAmps = hoodTalonFX.getStatorCurrent().getValueAsDouble();
		inputs.hoodBusVoltageVolts = hoodTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.hoodBusCurrentAmps = hoodTalonFX.getSupplyCurrent().getValueAsDouble();

		inputs.flywheelVelocityRPM = flywheelTalonFX.getVelocity().getValueAsDouble();
		inputs.flywheelVoltageVolts = flywheelTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.flywheelCurrentAmps = flywheelTalonFX.getStatorCurrent().getValueAsDouble();
		inputs.flywheelBusVoltageVolts = flywheelTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.flywheelBusCurrentAmps = flywheelTalonFX.getSupplyCurrent().getValueAsDouble();
	}

	public void actuate(ShooterInputs inputs, double turretPosition, double hoodPosition, double flywheelRPM) {
		Logger.recordOutput("/Shooter/turretErrorRadians", inputs.turretPosRadians - turretPosition);
		Logger.recordOutput("/Shooter/hoodErrorRadians", inputs.hoodPosRadians - hoodPosition);
		Logger.recordOutput("/Shooter/flywheelErrorRPM", inputs.flywheelVelocityRPM - flywheelRPM);

		if (!Robot.isReal()) return;

		turretTalonFX.setControl(turretControlRequest.withPosition(turretPosition));
		hoodTalonFX.setControl(hoodControlRequest.withPosition(hoodPosition));
		flywheelTalonFX.setControl(flywheelControlRequest.withVelocity(flywheelRPM));
		// follower automatically follows leader
	}
}
