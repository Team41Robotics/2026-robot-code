package frc.robot.subsystem.intake;

import static edu.wpi.first.math.MathUtil.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class IntakeHW {
	public static final double JOINT_RATIO = 1.0 / 27.0;

	public static final double JOINT_ENCODER_ZERO = 3.005 + PI / 3;

	// Spark heartbeat tuning (competition-safe): poll 5–20 Hz, timeout 100–200 ms
	public static final double SPARK_HEARTBEAT_PERIOD_Sec = 0.10;

	public SparkMax jointSparkMax;
	public RelativeEncoder jointEncoder;
	public SparkMax intakeSparkMax;
	public RelativeEncoder intakeEncoder;

	public CANcoder jointAbsoluteEncoder;

	public StatusSignal<Angle> jointAbsolutePositionSignal;

	private double jointLastHeartbeatPollTimeSec = Double.NEGATIVE_INFINITY;
	private double jointLastGoodTimeSec = Double.NEGATIVE_INFINITY;
	private int jointLastErrorCode = 0;

	private double intakeLastHeartbeatPollTimeSec = Double.NEGATIVE_INFINITY;
	private double intakeLastGoodTimeSec = Double.NEGATIVE_INFINITY;
	private int intakeLastErrorCode = 0;

	public void init() {
		if (!Robot.isReal()) return;

		double now = Timer.getTimestamp();
		jointLastHeartbeatPollTimeSec = now;
		jointLastGoodTimeSec = now;
		intakeLastHeartbeatPollTimeSec = now;
		intakeLastGoodTimeSec = now;

		jointSparkMax = new SparkMax(33, MotorType.kBrushless);
		SparkMaxConfig jointConfig = new SparkMaxConfig();
		jointConfig.inverted(true);
		jointConfig.encoder.positionConversionFactor(JOINT_RATIO * 2 * PI);
		jointConfig.encoder.velocityConversionFactor(JOINT_RATIO * 2 * PI / 60);
		jointConfig.smartCurrentLimit(20);
		jointConfig.secondaryCurrentLimit(40);
		jointConfig.idleMode(IdleMode.kBrake);
		jointConfig.voltageCompensation(12.0);
		jointSparkMax.configure(jointConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		jointEncoder = jointSparkMax.getEncoder();

		jointAbsoluteEncoder = new CANcoder(32);
		jointAbsoluteEncoder.clearStickyFaults();

		intakeSparkMax = new SparkMax(31, MotorType.kBrushless);
		SparkMaxConfig intakeConfig = new SparkMaxConfig();
		intakeConfig.inverted(false);
		intakeConfig.smartCurrentLimit(100);
		intakeConfig.secondaryCurrentLimit(120);
		intakeConfig.idleMode(IdleMode.kCoast);
		intakeConfig.voltageCompensation(12.0);
		intakeSparkMax.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		intakeEncoder = intakeSparkMax.getEncoder();

		jointAbsolutePositionSignal = jointAbsoluteEncoder.getPosition(false);
		jointAbsolutePositionSignal.setUpdateFrequency(50);
		jointAbsoluteEncoder.optimizeBusUtilization();
	}

	public void sense(IntakeInputs inputs) {
		if (!Robot.isReal()) return;

		double now = Timer.getTimestamp();
		// Force periodic active communication with each Spark.
		// This creates a real CAN transaction (response required), giving a real heartbeat.
		if ((now - jointLastHeartbeatPollTimeSec) >= SPARK_HEARTBEAT_PERIOD_Sec) {
			jointLastHeartbeatPollTimeSec = now;
			jointSparkMax.getMotorTemperature();
			REVLibError err = jointSparkMax.getLastError();
			jointLastErrorCode = err.value;
			if (err == REVLibError.kOk) {
				jointLastGoodTimeSec = now;
			}
		}
		if ((now - intakeLastHeartbeatPollTimeSec) >= SPARK_HEARTBEAT_PERIOD_Sec) {
			intakeLastHeartbeatPollTimeSec = now;
			intakeSparkMax.getMotorTemperature();
			REVLibError err = intakeSparkMax.getLastError();
			intakeLastErrorCode = err.value;
			if (err == REVLibError.kOk) {
				intakeLastGoodTimeSec = now;
			}
		}

		jointAbsolutePositionSignal.refresh();

		inputs.jointPosRadians = jointAbsolutePositionSignal.getValueAsDouble() * 2 * PI;
		inputs.jointPosRadians = angleModulus(inputs.jointPosRadians - JOINT_ENCODER_ZERO);
		jointEncoder.setPosition(inputs.jointPosRadians);
		inputs.jointAbsTsSec = jointAbsolutePositionSignal.getTimestamp().getTime();

		inputs.jointVelRadiansPerSec = jointEncoder.getVelocity();
		inputs.jointVoltageVolts = jointSparkMax.getBusVoltage() * jointSparkMax.getAppliedOutput();
		inputs.jointCurrentAmps = jointSparkMax.getOutputCurrent();
		inputs.jointBusVoltageVolts = jointSparkMax.getBusVoltage();

		inputs.intakeVelocityRPM = intakeEncoder.getVelocity();
		inputs.intakeVoltageVolts = intakeSparkMax.getBusVoltage() * intakeSparkMax.getAppliedOutput();
		inputs.intakeCurrentAmps = intakeSparkMax.getOutputCurrent();
		inputs.intakeBusVoltageVolts = intakeSparkMax.getBusVoltage();

		inputs.jointLastGoodTimeSec = jointLastGoodTimeSec;
		inputs.jointLastHeartbeatPollTimeSec = jointLastHeartbeatPollTimeSec;
		inputs.jointLastErrorCode = jointLastErrorCode;
		inputs.intakeLastGoodTimeSec = intakeLastGoodTimeSec;
		inputs.intakeLastHeartbeatPollTimeSec = intakeLastHeartbeatPollTimeSec;
		inputs.intakeLastErrorCode = intakeLastErrorCode;
	}

	public void actuate(IntakeInputs inputs, double jointVoltage, double intakeVoltage) {
		Logger.recordOutput("/Intake/jointCommandVoltageVolts", jointVoltage);

		if (!Robot.isReal()) return;

		jointSparkMax.setVoltage(jointVoltage);
		intakeSparkMax.setVoltage(intakeVoltage);
	}
}
