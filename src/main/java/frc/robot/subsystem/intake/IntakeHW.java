package frc.robot.subsystem.intake;

import static edu.wpi.first.math.MathUtil.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class IntakeHW {
	public static final double JOINT_RATIO = 1.0 / 27.0;

	public static final double JOINT_ENCODER_ZERO = 3.005 + PI / 3;

	public SparkMax jointSparkMax;
	public RelativeEncoder jointEncoder;
	public SparkMax intakeSparkMax;
	public RelativeEncoder intakeEncoder;

	public CANcoder jointAbsoluteEncoder;

	public StatusSignal<Angle> jointAbsolutePositionSignal;

	public void init() {
		if (!Robot.isReal()) return;

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
		intakeConfig.smartCurrentLimit(30);
		intakeConfig.secondaryCurrentLimit(60);
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

		jointAbsolutePositionSignal.refresh();

		inputs.jointPosRadians = jointAbsolutePositionSignal.getValueAsDouble() * 2 * PI;
		inputs.jointPosRadians = angleModulus(inputs.jointPosRadians - JOINT_ENCODER_ZERO);
		jointEncoder.setPosition(inputs.jointPosRadians);

		inputs.jointVelRadiansPerSec = jointEncoder.getVelocity();
		inputs.jointVoltageVolts = jointSparkMax.getBusVoltage() * jointSparkMax.getAppliedOutput();
		inputs.jointCurrentAmps = jointSparkMax.getOutputCurrent();
		inputs.jointBusVoltageVolts = jointSparkMax.getBusVoltage();

		inputs.intakeVelocityRPM = intakeEncoder.getVelocity();
		inputs.intakeVoltageVolts = intakeSparkMax.getBusVoltage() * intakeSparkMax.getAppliedOutput();
		inputs.intakeCurrentAmps = intakeSparkMax.getOutputCurrent();
		inputs.intakeBusVoltageVolts = intakeSparkMax.getBusVoltage();
	}

	public void actuate(IntakeInputs inputs, double jointVoltage, double intakeVoltage) {
		Logger.recordOutput("/Intake/jointCommandVoltageVolts", jointVoltage);

		if (!Robot.isReal()) return;

		jointSparkMax.setVoltage(jointVoltage);
		intakeSparkMax.setVoltage(intakeVoltage);
	}
}
