package frc.robot.subsystem.drive;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class SwerveHW {
	public static final double DRIVE_RATIO = 1 / 5.36; // motor * ratio = mechanism // FIXME.
	public static final double TURN_RATIO = 1 / 18.75; // FIXME.
	public static final double WHEEL_RAD = 2 * 2.54 / 100.; // FIXME. wheel radius (m)

	public static final double DRIVE_kP = 4; // FIXME. drive PID P
	public static final double TURN_kP = 20; // FIXME. turn PID P
	public static final double TURN_kD = 0.4; // FIXME. turn PID D

	public TalonFX driveTalonFX;
	public TalonFX turnTalonFX;
	public CANcoder turnAbsoluteEncoder;

	public String logRoot;
	public double angleOffset = 0;

	public boolean sysIdDrive = false;
	public boolean sysIdTurn = false;

	public VelocityVoltage driveControlRequest = new VelocityVoltage(0).withSlot(0);
	public PositionVoltage turnControlRequest = new PositionVoltage(0).withSlot(0);

	public void init(SwerveModuleConfiguration config) {
		logRoot = "Swerve/" + config.name;
		angleOffset = config.angleOffset;

		if (!Robot.isReal()) return;

		driveTalonFX = new TalonFX(config.driveMotorId, driveBus);
		TalonFXConfiguration driveConfig = new TalonFXConfiguration();

		driveConfig.Slot0.kP = DRIVE_kP * DRIVE_RATIO * 2 * PI * WHEEL_RAD;

		driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		driveConfig.CurrentLimits.SupplyCurrentLimit = 60; // FIXME. supply current limit (A)
		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		driveConfig.CurrentLimits.StatorCurrentLimit = 23; // FIXME. stator current limit (A)

		driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		driveTalonFX.getConfigurator().apply(driveConfig);
		driveTalonFX.clearStickyFaults();
		driveTalonFX.setPosition(0);
		driveTalonFX.setNeutralMode(NeutralModeValue.Coast);

		turnTalonFX = new TalonFX(config.turnMotorId, driveBus);
		TalonFXConfiguration turnConfig = new TalonFXConfiguration();

		turnConfig.Slot0.kP = TURN_kP * TURN_RATIO * 2 * PI;
		turnConfig.Slot0.kD = TURN_kD * TURN_RATIO * 2 * PI;

		turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		turnConfig.CurrentLimits.SupplyCurrentLimit = 30; // FIXME. supply current limit (A)

		turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		turnTalonFX.getConfigurator().apply(turnConfig);
		turnTalonFX.clearStickyFaults();
		turnTalonFX.setNeutralMode(NeutralModeValue.Coast);

		turnAbsoluteEncoder = new CANcoder(config.encoderId, driveBus);
		turnAbsoluteEncoder.clearStickyFaults();
	}

	public void sense(SwerveInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.drivePosMeters = driveTalonFX.getPosition().getValueAsDouble() * 2 * PI * DRIVE_RATIO * WHEEL_RAD;
		inputs.driveVelMetersPerSec = driveTalonFX.getVelocity().getValueAsDouble() * 2 * PI * DRIVE_RATIO * WHEEL_RAD;

		inputs.driveBusVoltageVolts = driveTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.driveBusCurrentAmps = driveTalonFX.getSupplyCurrent().getValueAsDouble();
		inputs.driveVoltageVolts = driveTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.driveCurrentAmps = driveTalonFX.getStatorCurrent().getValueAsDouble();

		inputs.turnBusVoltageVolts = turnTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.turnBusCurrentAmps = turnTalonFX.getSupplyCurrent().getValueAsDouble();
		inputs.turnPosRadians = turnTalonFX.getPosition().getValueAsDouble() * 2 * PI * TURN_RATIO;
		inputs.turnVelRadiansPerSec = turnTalonFX.getVelocity().getValueAsDouble() * 2 * PI * TURN_RATIO;

		inputs.turnVoltageVolts = turnTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.turnCurrentAmps = turnTalonFX.getStatorCurrent().getValueAsDouble();

		inputs.turnAbsPosRadians = turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * PI - angleOffset;
	}

	public void actuate(SwerveInputs inputs, double targetVel, double driveFF, double targetAng, double turnFF) {
		Logger.recordOutput(logRoot + "/targetVelMetersPerSec", targetVel);
		Logger.recordOutput(logRoot + "/driveFFVolts", driveFF);
		Logger.recordOutput(logRoot + "/targetAngRadians", angleModulus(targetAng));
		Logger.recordOutput(logRoot + "/turnFFVolts", turnFF);

		if (!Robot.isReal()) return;

		if (!sysIdDrive && !sysIdTurn) {
			driveTalonFX.setControl(driveControlRequest
					.withVelocity(targetVel / (DRIVE_RATIO * 2 * PI * WHEEL_RAD))
					.withFeedForward(driveFF));
		}

		Logger.recordOutput(logRoot + "/driveErrorMetersPerSec", inputs.driveVelMetersPerSec - targetVel);

		double diff = angleModulus(targetAng - inputs.turnAbsPosRadians);
		Logger.recordOutput(logRoot + "/turnErrorRadians", diff);

		if (!sysIdTurn) {
			turnTalonFX.setControl(turnControlRequest
					.withPosition((inputs.turnPosRadians + diff) / (2 * PI * TURN_RATIO))
					.withFeedForward(turnFF));
		}
	}
}
