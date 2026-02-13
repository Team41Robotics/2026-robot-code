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
	public static double DRIVE_RATIO = 1 / 5.36; // gear ratios are motor * gear ratio = mechanism // FIXME.
	public static double TURN_RATIO = 1 / 18.75; // FIXME.
	public static double SWERVE_WHEEL_RAD = 2 * 2.54 / 100.; // FIXME.

	public static double DRIVE_kP = 4; // FIXME. drive PID P

	public static double TURN_kP = 20; // FIXME. turn PID P
	public static double TURN_kD = 0.4; // FIXME. turn PID D

	public TalonFX driveTalonFX;
	public TalonFX turnTalonFX;
	public CANcoder turnAbsoluteEncoder;

	public String logRoot;

	public double angleOffset = 0;

	public boolean sysidDrive = false;
	public boolean sysidTurn = false;

	public void init(SwerveModuleConfiguration config) {
		logRoot = "Swerve/" + config.name;
		angleOffset = config.angleOffset;

		if (!Robot.isReal()) return;

		driveTalonFX = new TalonFX(config.drive_motor_id, driveBus);
		TalonFXConfiguration driveConfig = new TalonFXConfiguration();

		driveConfig.Slot0.kP = DRIVE_kP * DRIVE_RATIO * 2 * PI * SWERVE_WHEEL_RAD;

		driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		driveConfig.CurrentLimits.SupplyCurrentLimit = 60; // FIXME. supply current limit (A)
		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		driveConfig.CurrentLimits.StatorCurrentLimit = 23; // FIXME. stator current limit (A)

		driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		driveTalonFX.getConfigurator().apply(driveConfig);
		driveTalonFX.clearStickyFaults();
		driveTalonFX.setPosition(0);
		driveTalonFX.setNeutralMode(NeutralModeValue.Coast);

		turnTalonFX = new TalonFX(config.turn_motor_id, driveBus);
		TalonFXConfiguration turnConfig = new TalonFXConfiguration();

		turnConfig.Slot0.kP = TURN_kP * TURN_RATIO * 2 * PI;
		turnConfig.Slot0.kD = TURN_kD * TURN_RATIO * 2 * PI;

		turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		turnConfig.CurrentLimits.SupplyCurrentLimit = 30; // FIXME. supply current limit (A)

		turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		turnTalonFX.getConfigurator().apply(turnConfig);
		turnTalonFX.clearStickyFaults();
		turnTalonFX.setNeutralMode(NeutralModeValue.Coast);

		turnAbsoluteEncoder = new CANcoder(config.encoder_id, driveBus);
		turnAbsoluteEncoder.clearStickyFaults();
	}

	public void sense(SwerveInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.drivePos = driveTalonFX.getPosition().getValueAsDouble() * 2 * PI * DRIVE_RATIO * SWERVE_WHEEL_RAD;
		inputs.driveVel = driveTalonFX.getVelocity().getValueAsDouble() * 2 * PI * DRIVE_RATIO * SWERVE_WHEEL_RAD;

		inputs.driveBusVoltage = driveTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.driveBusCurrent = driveTalonFX.getSupplyCurrent().getValueAsDouble();
		inputs.driveVoltage = driveTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.driveCurrent = driveTalonFX.getStatorCurrent().getValueAsDouble();

		inputs.turnBusVoltage = turnTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.turnBusCurrent = turnTalonFX.getSupplyCurrent().getValueAsDouble();
		inputs.turnPos = turnTalonFX.getPosition().getValueAsDouble() * 2 * PI * TURN_RATIO;
		inputs.turnVel = turnTalonFX.getVelocity().getValueAsDouble() * 2 * PI * TURN_RATIO;

		inputs.turnVoltage = turnTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.turnCurrent = turnTalonFX.getStatorCurrent().getValueAsDouble();

		inputs.turnAbsPos = turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * PI - angleOffset;
	}

	public void actuate(SwerveInputs inputs, double driveVelocity, double driveFF, double turnPosition, double turnFF) {
		Logger.recordOutput(logRoot + "/actuatedDriveVel", driveVelocity);
		Logger.recordOutput(logRoot + "/actuatedDriveFF", driveFF);
		Logger.recordOutput(logRoot + "/actuatedTurnPos", angleModulus(turnPosition));
		Logger.recordOutput(logRoot + "/actuatedTurnFF", turnFF);

		if (!Robot.isReal()) return;

		if (!sysidDrive && !sysidTurn) {
			driveTalonFX.setControl(new VelocityVoltage(driveVelocity / (DRIVE_RATIO * 2 * PI * SWERVE_WHEEL_RAD))
					.withFeedForward(driveFF)
					.withSlot(0));
		}

		Logger.recordOutput(logRoot + "/driveError", inputs.driveVel - driveVelocity);

		double diff = angleModulus(turnPosition - inputs.turnAbsPos);
		Logger.recordOutput(logRoot + "/turnError", diff);

		if (!sysidTurn) {
			turnTalonFX.setControl(new PositionVoltage((inputs.turnPos + diff) / (2 * PI * TURN_RATIO))
					.withFeedForward(turnFF)
					.withSlot(0));
		}
	}
}
