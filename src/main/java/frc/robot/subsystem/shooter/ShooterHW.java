package frc.robot.subsystem.shooter;

import static edu.wpi.first.math.MathUtil.*;
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
	public static final double TURRET_RATIO = 1.0 / 16.8571667;
	public static final double HOOD_RATIO = 1.0 / 3.0 / 17.5;
	public static final double FLYWHEEL_RATIO = 1.0;

	public static final double TURRET_kP = 128.0;
	public static final double TURRET_kI = 36.0;
	public static final double TURRET_kD = 0.0;

	public static final double HOOD_kP = 192.0;
	public static final double HOOD_kI = 96.0;
	public static final double HOOD_kD = 0.0; // TUNEME

	public static final double FLYWHEEL_kP = 0.63934; // TUNEME
	public static final double FLYWHEEL_kD = 0; // TUNEME
	public static final double FLYWHEEL_kV = 0.11494;
	public static final double FLYWHEEL_kS = 0.24333;

	public static final double TURRET_START_POS = -PI / 2;
	public static double K = PI - 14 / 180.0 * PI;
	public static double ENCODER_LIM_POS1RIGHT = angleModulus(-(-95.3 - 95.903) / 180.0 * PI + K);
	public static double ENCODER_LIM_POS1LEFT = angleModulus(-(-87.7 - 95.903) / 180.0 * PI + K);
	public static double ENCODER_LIM_POS2RIGHT = angleModulus(-(0.2 - 95.903) / 180.0 * PI + K);
	public static double ENCODER_LIM_POS2LEFT = angleModulus(-(8.1 - 95.903) / 180.0 * PI + K);
	public static double ENCODER_LIM_POS3RIGHT = angleModulus(-(-183.3 - 95.903) / 180.0 * PI + K);
	public static double ENCODER_LIM_POS3LEFT = angleModulus(-(-177.3 - 95.903) / 180.0 * PI + K);

	public TalonFX turretTalonFX;
	public TalonFX hoodTalonFX;
	public TalonFX flywheelTalonFX;
	public TalonFX flywheelFollowerTalonFX;

	public PositionVoltage turretControlRequest = new PositionVoltage(0).withSlot(0);
	public PositionVoltage hoodControlRequest = new PositionVoltage(0).withSlot(0);
	public VelocityVoltage flywheelControlRequest = new VelocityVoltage(0).withSlot(0);
	public boolean sysIdFlywheel = false;

	public DigitalInput hoodLimitSwitch;
	public DigitalInput turretLimitSwitch;

	public ShooterHW() {
		if (ENCODER_LIM_POS1LEFT > ENCODER_LIM_POS1RIGHT) {
			double temp = ENCODER_LIM_POS1LEFT;
			ENCODER_LIM_POS1LEFT = ENCODER_LIM_POS1RIGHT;
			ENCODER_LIM_POS1RIGHT = temp;
		}
		if (ENCODER_LIM_POS2LEFT > ENCODER_LIM_POS2RIGHT) {
			double temp = ENCODER_LIM_POS2LEFT;
			ENCODER_LIM_POS2LEFT = ENCODER_LIM_POS2RIGHT;
			ENCODER_LIM_POS2RIGHT = temp;
		}
		if (ENCODER_LIM_POS3LEFT > ENCODER_LIM_POS3RIGHT) {
			double temp = ENCODER_LIM_POS3LEFT;
			ENCODER_LIM_POS3LEFT = ENCODER_LIM_POS3RIGHT;
			ENCODER_LIM_POS3RIGHT = temp;
		}
	}

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
		turretConfig.CurrentLimits.SupplyCurrentLimit = 40;
		turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		turretConfig.CurrentLimits.StatorCurrentLimit = 60;
		turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		turretTalonFX.getConfigurator().apply(turretConfig);
		turretTalonFX.clearStickyFaults();
		turretTalonFX.setPosition(0);
		turretTalonFX.setNeutralMode(NeutralModeValue.Brake);

		turretLimitSwitch = new DigitalInput(0);

		// --- Hood ---
		hoodTalonFX = new TalonFX(52);
		TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
		hoodConfig.Feedback.SensorToMechanismRatio = 1.0 / (HOOD_RATIO * 2 * PI);
		hoodConfig.Slot0.kP = HOOD_kP;
		hoodConfig.Slot0.kI = HOOD_kI;
		hoodConfig.Slot0.kD = HOOD_kD;
		hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		hoodConfig.CurrentLimits.SupplyCurrentLimit = 40;
		hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		hoodConfig.CurrentLimits.StatorCurrentLimit = 60;
		hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		hoodTalonFX.getConfigurator().apply(hoodConfig);
		hoodTalonFX.clearStickyFaults();
		hoodTalonFX.setPosition(0);
		hoodTalonFX.setNeutralMode(NeutralModeValue.Brake);

		hoodLimitSwitch = new DigitalInput(1);

		// --- Flywheel (leader) ---
		flywheelTalonFX = new TalonFX(53);
		TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
		flywheelConfig.Feedback.SensorToMechanismRatio = 1.0 / FLYWHEEL_RATIO;
		flywheelConfig.Slot0.kP = FLYWHEEL_kP;
		flywheelConfig.Slot0.kV = FLYWHEEL_kV;
		flywheelConfig.Slot0.kS = FLYWHEEL_kS;
		flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		flywheelConfig.CurrentLimits.SupplyCurrentLimit = 80; // TUNEME
		flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		flywheelConfig.CurrentLimits.StatorCurrentLimit = 180; // TUNEME
		flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		flywheelTalonFX.getConfigurator().apply(flywheelConfig);
		flywheelTalonFX.clearStickyFaults();
		flywheelTalonFX.setNeutralMode(NeutralModeValue.Coast);

		// --- Flywheel follower ---
		flywheelFollowerTalonFX = new TalonFX(51);
		flywheelFollowerTalonFX.getConfigurator().apply(new TalonFXConfiguration());
		flywheelFollowerTalonFX.clearStickyFaults();
		flywheelFollowerTalonFX.setNeutralMode(NeutralModeValue.Coast);
		flywheelFollowerTalonFX.setControl(new Follower(53, MotorAlignmentValue.Opposed));
	}

	public void sense(ShooterInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.turretLimitSwitchOn = !turretLimitSwitch.get();
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
			if (abs(inputs.turretPosRadians - ENCODER_LIM_POS3LEFT) < abs(inputs.turretPosRadians - bestPos)) {
				bestPos = ENCODER_LIM_POS3LEFT;
			}
			if (abs(inputs.turretPosRadians - ENCODER_LIM_POS3RIGHT) < abs(inputs.turretPosRadians - bestPos)) {
				bestPos = ENCODER_LIM_POS3RIGHT;
			}
			if (ENCODER_LIM_POS3LEFT < inputs.turretPosRadians && inputs.turretPosRadians < ENCODER_LIM_POS3RIGHT) {
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

		inputs.isHoodLimitSwitchOn = !hoodLimitSwitch.get();
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

		inputs.flywheelVelocityRPM = flywheelTalonFX.getVelocity().getValueAsDouble() * 60;
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
		if (!sysIdFlywheel) flywheelTalonFX.setControl(flywheelControlRequest.withVelocity(flywheelRPM / 60.0));
	}
}
