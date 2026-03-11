package frc.robot.subsystem.drive;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class SwerveHW {
	public static final double DRIVE_RATIO = 1 / 5.27; // motor * ratio = mechanism
	public static final double TURN_RATIO = 11.0 / 287;
	public static final double WHEEL_RAD = 2 * 2.54 / 100.;

	public static final double DRIVE_kP = 4; // TUNEME. drive PID P
	public static final double TURN_kP = 20; // TUNEME. turn PID P
	public static final double TURN_kD = 0.4; // TUNEME. turn PID D

	public TalonFX driveTalonFX;
	public TalonFX turnTalonFX;
	public CANcoder turnAbsoluteEncoder;

	public String logRoot;
	public double angleOffset = 0;

	public boolean sysIdDrive = false;
	public boolean sysIdTurn = false;

	public VelocityVoltage driveControlRequest = new VelocityVoltage(0).withSlot(0);
	public PositionVoltage turnControlRequest = new PositionVoltage(0).withSlot(0);

	// Cached StatusSignals — drive motor
	public StatusSignal<Angle> drivePosition;
	public StatusSignal<AngularVelocity> driveVelocity;
	public StatusSignal<Voltage> driveSupplyVoltage;
	public StatusSignal<Current> driveSupplyCurrent;
	public StatusSignal<Voltage> driveMotorVoltage;
	public StatusSignal<Current> driveStatorCurrent;

	// Cached StatusSignals — turn motor
	public StatusSignal<Voltage> turnSupplyVoltage;
	public StatusSignal<Current> turnSupplyCurrent;
	public StatusSignal<Angle> turnPosition;
	public StatusSignal<AngularVelocity> turnVelocity;
	public StatusSignal<Voltage> turnMotorVoltage;
	public StatusSignal<Current> turnStatorCurrent;

	// Cached StatusSignals — CANcoder
	public StatusSignal<Angle> turnAbsolutePosition;

	public void init(SwerveModuleConfiguration config) {
		logRoot = "Swerve/" + config.name;
		angleOffset = config.angleOffset;

		if (!Robot.isReal()) return;

		driveTalonFX = new TalonFX(config.driveMotorId, driveBus);
		TalonFXConfiguration driveConfig = new TalonFXConfiguration();
		driveConfig.Feedback.SensorToMechanismRatio = 1.0 / (DRIVE_RATIO * 2 * PI * WHEEL_RAD);
		driveConfig.Slot0.kP = DRIVE_kP;
		driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		driveConfig.CurrentLimits.SupplyCurrentLimit = 40;
		driveConfig.CurrentLimits.SupplyCurrentLowerTime = 2.0;
		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		driveConfig.CurrentLimits.StatorCurrentLimit = 90;
		driveConfig.Voltage.PeakForwardVoltage = 12.0;
		driveConfig.Voltage.PeakReverseVoltage = -12.0;
		driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		driveTalonFX.getConfigurator().apply(driveConfig);
		driveTalonFX.clearStickyFaults();
		driveTalonFX.setPosition(0);
		driveTalonFX.setNeutralMode(NeutralModeValue.Coast);

		turnTalonFX = new TalonFX(config.turnMotorId, driveBus);
		TalonFXConfiguration turnConfig = new TalonFXConfiguration();
		turnConfig.Feedback.SensorToMechanismRatio = 1.0 / (TURN_RATIO * 2 * PI);
		turnConfig.Slot0.kP = TURN_kP;
		turnConfig.Slot0.kD = TURN_kD;
		turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		turnConfig.CurrentLimits.SupplyCurrentLimit = 20;
		turnConfig.Voltage.PeakForwardVoltage = 12.0;
		turnConfig.Voltage.PeakReverseVoltage = -12.0;
		turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		turnTalonFX.getConfigurator().apply(turnConfig);
		turnTalonFX.clearStickyFaults();
		turnTalonFX.setNeutralMode(NeutralModeValue.Brake);

		turnAbsoluteEncoder = new CANcoder(config.encoderId, driveBus);
		turnAbsoluteEncoder.clearStickyFaults();

		// Initialize cached signals
		drivePosition = driveTalonFX.getPosition(false);
		driveVelocity = driveTalonFX.getVelocity(false);
		driveSupplyVoltage = driveTalonFX.getSupplyVoltage(false);
		driveSupplyCurrent = driveTalonFX.getSupplyCurrent(false);
		driveMotorVoltage = driveTalonFX.getMotorVoltage(false);
		driveStatorCurrent = driveTalonFX.getStatorCurrent(false);

		turnSupplyVoltage = turnTalonFX.getSupplyVoltage(false);
		turnSupplyCurrent = turnTalonFX.getSupplyCurrent(false);
		turnPosition = turnTalonFX.getPosition(false);
		turnVelocity = turnTalonFX.getVelocity(false);
		turnMotorVoltage = turnTalonFX.getMotorVoltage(false);
		turnStatorCurrent = turnTalonFX.getStatorCurrent(false);

		turnAbsolutePosition = turnAbsoluteEncoder.getAbsolutePosition(false);

		// Set update frequencies
		drivePosition.setUpdateFrequency(50);
		driveVelocity.setUpdateFrequency(50);
		driveSupplyVoltage.setUpdateFrequency(50);
		driveSupplyCurrent.setUpdateFrequency(50);
		driveMotorVoltage.setUpdateFrequency(50);
		driveStatorCurrent.setUpdateFrequency(50);

		turnSupplyVoltage.setUpdateFrequency(50);
		turnSupplyCurrent.setUpdateFrequency(50);
		turnPosition.setUpdateFrequency(50);
		turnVelocity.setUpdateFrequency(50);
		turnMotorVoltage.setUpdateFrequency(50);
		turnStatorCurrent.setUpdateFrequency(50);

		turnAbsolutePosition.setUpdateFrequency(50);

		// Optimize bus utilization — disable signals we don't use
		driveTalonFX.optimizeBusUtilization();
		turnTalonFX.optimizeBusUtilization();
		turnAbsoluteEncoder.optimizeBusUtilization();
	}

	public void sense(SwerveInputs inputs) {
		if (!Robot.isReal()) return;

		BaseStatusSignal.waitForAll(
				0,
				drivePosition,
				driveVelocity,
				driveSupplyVoltage,
				driveSupplyCurrent,
				driveMotorVoltage,
				driveStatorCurrent,
				turnSupplyVoltage,
				turnSupplyCurrent,
				turnPosition,
				turnVelocity,
				turnMotorVoltage,
				turnStatorCurrent,
				turnAbsolutePosition);

		inputs.drivePosMeters = drivePosition.getValueAsDouble();
		inputs.driveVelMetersPerSec = driveVelocity.getValueAsDouble();

		inputs.driveBusVoltageVolts = driveSupplyVoltage.getValueAsDouble();
		inputs.driveBusCurrentAmps = driveSupplyCurrent.getValueAsDouble();
		inputs.driveVoltageVolts = driveMotorVoltage.getValueAsDouble();
		inputs.driveCurrentAmps = driveStatorCurrent.getValueAsDouble();

		inputs.turnBusVoltageVolts = turnSupplyVoltage.getValueAsDouble();
		inputs.turnBusCurrentAmps = turnSupplyCurrent.getValueAsDouble();
		inputs.turnPosRadians = turnPosition.getValueAsDouble();
		inputs.turnVelRadiansPerSec = turnVelocity.getValueAsDouble();

		inputs.turnVoltageVolts = turnMotorVoltage.getValueAsDouble();
		inputs.turnCurrentAmps = turnStatorCurrent.getValueAsDouble();

		inputs.turnAbsPosRadians = turnAbsolutePosition.getValueAsDouble() * 2 * PI;
		inputs.turnAbsPosRadians = angleModulus(inputs.turnAbsPosRadians - angleOffset);
	}

	public void actuate(SwerveInputs inputs, double targetVel, double driveFF, double targetAng, double turnFF) {
		if (!Robot.isReal()) return;

		if (!sysIdDrive && !sysIdTurn) {
			driveTalonFX.setControl(driveControlRequest.withVelocity(targetVel).withFeedForward(driveFF));
		}

		Logger.recordOutput(logRoot + "/driveErrorMetersPerSec", inputs.driveVelMetersPerSec - targetVel);

		double diff = angleModulus(targetAng - inputs.turnAbsPosRadians);
		Logger.recordOutput(logRoot + "/turnErrorRadians", diff);

		if (!sysIdTurn) {
			turnTalonFX.setControl(turnControlRequest
					.withPosition(inputs.turnPosRadians + diff)
					.withFeedForward(turnFF));
		}
	}
}
