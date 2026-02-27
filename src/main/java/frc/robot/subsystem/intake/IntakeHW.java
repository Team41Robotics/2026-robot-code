package frc.robot.subsystem.intake;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static java.lang.Math.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class IntakeHW {
	public static final double JOINT_RATIO = 1.0 / 27.0; // FIXME. joint gear ratio (motor * ratio = mechanism)
	public static final double JOINT_kP = 10.0; // FIXME. joint PID P
	public static final double JOINT_kD = 0.0; // FIXME. joint PID D

	public static final double JOINT_ENCODER_ZERO = 0; // FIXME. absolute encoder zero (rad)

	public SparkMax jointSparkMax;
	public RelativeEncoder jointEncoder;
	public TalonFX intakeTalonFX;

	public CANcoder jointAbsoluteEncoder;

	public VoltageOut intakeControlRequest = new VoltageOut(0);

	public void init() {
		if (!Robot.isReal()) return;

		jointSparkMax = new SparkMax(32, MotorType.kBrushless);
		jointEncoder = jointSparkMax.getEncoder();
		SparkMaxConfig jointConfig = new SparkMaxConfig();
		jointConfig.encoder.positionConversionFactor(JOINT_RATIO * 2 * PI);
		jointConfig.encoder.velocityConversionFactor(JOINT_RATIO * 2 * PI / 60);
		jointConfig.closedLoop.p(JOINT_kP).d(JOINT_kD);
		jointConfig.smartCurrentLimit(40, 60);
		jointConfig.inverted(true); // FIXME. inversion
		jointConfig.idleMode(IdleMode.kBrake);
		jointSparkMax.configure(jointConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		jointAbsoluteEncoder = new CANcoder(33);
		jointAbsoluteEncoder.clearStickyFaults();

		intakeTalonFX = new TalonFX(31);
		TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
		intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.SupplyCurrentLimit = 40; // FIXME. supply current limit (A)
		intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // FIXME. inversion
		intakeTalonFX.getConfigurator().apply(intakeConfig);
		intakeTalonFX.clearStickyFaults();
		intakeTalonFX.setNeutralMode(NeutralModeValue.Coast);
	}

	public void sense(IntakeInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.jointPosRadians = jointAbsoluteEncoder.getPosition().getValueAsDouble() * JOINT_RATIO * 2 * PI;
		inputs.jointPosRadians = angleModulus(inputs.jointPosRadians - JOINT_ENCODER_ZERO);

		inputs.jointVelRadiansPerSec = jointEncoder.getVelocity();
		inputs.jointVoltageVolts = jointSparkMax.getBusVoltage() * jointSparkMax.getAppliedOutput();
		inputs.jointCurrentAmps = jointSparkMax.getOutputCurrent();
		inputs.jointBusVoltageVolts = jointSparkMax.getBusVoltage();

		inputs.intakeVelocityRPM = intakeTalonFX.getVelocity().getValueAsDouble() * 60.0;
		inputs.intakeVoltageVolts = intakeTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.intakeCurrentAmps = intakeTalonFX.getStatorCurrent().getValueAsDouble();
		inputs.intakeBusVoltageVolts = intakeTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.intakeBusCurrentAmps = intakeTalonFX.getSupplyCurrent().getValueAsDouble();
	}

	public void actuate(IntakeInputs inputs, double jointPosition, double intakeVoltage) {
		Logger.recordOutput("/Intake/jointErrorRadians", inputs.jointPosRadians - jointPosition);

		if (!Robot.isReal()) return;

		jointSparkMax.getClosedLoopController().setSetpoint(jointPosition, ControlType.kPosition);
		intakeTalonFX.setControl(intakeControlRequest.withOutput(intakeVoltage));
	}

	public void zeroJointPosition() {
		if (!Robot.isReal()) return;

		jointEncoder.setPosition(0);
	}
}
