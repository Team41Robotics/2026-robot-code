package frc.robot.subsystem.drive;

import static java.lang.Math.PI;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.Robot;

@AutoLog
public class SwerveHW {

	public static double DRIVE_RATIO = 1 / 5.36; // gear ratios are motor * gear ratio = mechanism
	public static double TURN_RATIO = 1 / 18.75;
	public static double SWERVE_WHEEL_RAD = 2 * 2.54;

	public static double DRIVE_kP = 0;

	public static double TURN_kP = 4.0951;
	public static double TURN_kD = 0.68251;

	public TalonFX driveTalonFX;
	public TalonFX turnTalonFX;
	public CANcoder turnAbsoluteEncoder;

	public String logRoot;

	public double angleOffset = 0;

	public void init(SwerveModuleConfiguration config) {
		logRoot = "SwerveHW/" + config.name;
		angleOffset = config.angleOffset;

		if (!Robot.isReal()) return;

		driveTalonFX = new TalonFX(config.drive_motor_id, "Ducky");
		TalonFXConfiguration driveConfig = new TalonFXConfiguration();

		driveConfig.Slot0.kP = DRIVE_kP * DRIVE_RATIO * 2 * PI * SWERVE_WHEEL_RAD;

		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		driveConfig.CurrentLimits.withStatorCurrentLimit(120);

		driveTalonFX.getConfigurator().apply(driveConfig);
		driveTalonFX.clearStickyFaults();
		driveTalonFX.setPosition(0);
		driveTalonFX.setNeutralMode(NeutralModeValue.Brake);

		turnTalonFX = new TalonFX(config.turn_motor_id, "Ducky");
		TalonFXConfiguration turnConfig = new TalonFXConfiguration();

		turnConfig.Slot0.kP = TURN_kP * TURN_RATIO * 2 * PI;

		turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		turnConfig.CurrentLimits.withStatorCurrentLimit(80);

		turnTalonFX.getConfigurator().apply(turnConfig);
		turnTalonFX.clearStickyFaults();
		turnTalonFX.setNeutralMode(NeutralModeValue.Brake);

		turnAbsoluteEncoder = new CANcoder(config.encoder_id, "Ducky");
		turnAbsoluteEncoder.clearStickyFaults();
	}

	public double drivePos;
	public double driveVel;
	public double driveVoltage;
	public double driveCurrent;

	public double turnPos;
	public double turnVel;
	public double turnVoltage;
	public double turnCurrent;

	public double turnAbsPos;

	public void sense() {
		if (!Robot.isReal()) return;

		drivePos = driveTalonFX.getPosition().getValueAsDouble() * 2 * PI * DRIVE_RATIO * SWERVE_WHEEL_RAD;
		driveVel = driveTalonFX.getVelocity().getValueAsDouble() * 2 * PI * DRIVE_RATIO * SWERVE_WHEEL_RAD;

		driveVoltage = driveTalonFX.getMotorVoltage().getValueAsDouble();
		driveCurrent = driveTalonFX.getStatorCurrent().getValueAsDouble();

		turnPos = turnTalonFX.getPosition().getValueAsDouble() / TURN_RATIO;
		turnVel = turnTalonFX.getVelocity().getValueAsDouble() * 2 * PI * TURN_RATIO;

		turnVoltage = turnTalonFX.getMotorVoltage().getValueAsDouble();
		turnCurrent = turnTalonFX.getStatorCurrent().getValueAsDouble();

		turnAbsPos = turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() - angleOffset;
	}

	public void actuate(double driveVelocity, double driveFF, double turnPosition, double turnFF) {
		Logger.recordOutput(logRoot + "/actuatedDriveVel", driveVelocity);
		Logger.recordOutput(logRoot + "/actuatedDriveFF", driveFF);
		Logger.recordOutput(logRoot + "/actuatedTurnPos", turnPosition);
		Logger.recordOutput(logRoot + "/actuatedTurnFF", turnFF);

		if (!Robot.isReal()) return;

		driveTalonFX.setControl(new VelocityVoltage(driveVelocity / (DRIVE_RATIO * 2 * PI * SWERVE_WHEEL_RAD))
				.withFeedForward(driveFF)
				.withSlot(0));

		double diff = MathUtil.angleModulus(turnPosition - turnAbsPos);
		turnTalonFX.setControl(
				new PositionVoltage(turnPos + diff).withFeedForward(turnFF).withSlot(0));
	}
}
