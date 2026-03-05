package frc.robot.subsystem.intake;

import static edu.wpi.first.math.MathUtil.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class IntakeHW {
	public static final double JOINT_RATIO = 1.0 / 27.0;

	public static final double JOINT_ENCODER_ZERO = 3.005 + PI / 3;

	public SparkMax jointSparkMax;
	public RelativeEncoder jointEncoder;
	public TalonFX intakeTalonFX;

	public CANcoder jointAbsoluteEncoder;

	public StatusSignal<Angle> jointAbsolutePositionSignal;
	public StatusSignal<AngularVelocity> intakeVelocitySignal;
	public StatusSignal<Voltage> intakeMotorVoltageSignal;
	public StatusSignal<Current> intakeStatorCurrentSignal;
	public StatusSignal<Voltage> intakeSupplyVoltageSignal;
	public StatusSignal<Current> intakeSupplyCurrentSignal;

	public VoltageOut intakeControlRequest = new VoltageOut(0);

	public void init() {
		if (!Robot.isReal()) return;

		jointSparkMax = new SparkMax(33, MotorType.kBrushless);
		SparkMaxConfig jointConfig = new SparkMaxConfig();
		jointConfig.inverted(true);
		jointConfig.encoder.positionConversionFactor(JOINT_RATIO * 2 * PI);
		jointConfig.encoder.velocityConversionFactor(JOINT_RATIO * 2 * PI / 60);
		jointConfig.smartCurrentLimit(60, 60);
		jointConfig.idleMode(IdleMode.kBrake);
		jointSparkMax.configure(jointConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		jointEncoder = jointSparkMax.getEncoder();

		jointAbsoluteEncoder = new CANcoder(32);
		jointAbsoluteEncoder.clearStickyFaults();

		intakeTalonFX = new TalonFX(31);
		TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
		intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.SupplyCurrentLimit = 60;
		intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.StatorCurrentLimit = 120;
		intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		intakeTalonFX.getConfigurator().apply(intakeConfig);
		intakeTalonFX.clearStickyFaults();
		intakeTalonFX.setNeutralMode(NeutralModeValue.Coast);

		jointAbsolutePositionSignal = jointAbsoluteEncoder.getPosition(false);
		intakeVelocitySignal = intakeTalonFX.getVelocity(false);
		intakeMotorVoltageSignal = intakeTalonFX.getMotorVoltage(false);
		intakeStatorCurrentSignal = intakeTalonFX.getStatorCurrent(false);
		intakeSupplyVoltageSignal = intakeTalonFX.getSupplyVoltage(false);
		intakeSupplyCurrentSignal = intakeTalonFX.getSupplyCurrent(false);

		jointAbsolutePositionSignal.setUpdateFrequency(50);
		intakeVelocitySignal.setUpdateFrequency(50);
		intakeMotorVoltageSignal.setUpdateFrequency(50);
		intakeStatorCurrentSignal.setUpdateFrequency(50);
		intakeSupplyVoltageSignal.setUpdateFrequency(50);
		intakeSupplyCurrentSignal.setUpdateFrequency(50);

		jointAbsoluteEncoder.optimizeBusUtilization();
		intakeTalonFX.optimizeBusUtilization();
	}

	public void sense(IntakeInputs inputs) {
		if (!Robot.isReal()) return;

		BaseStatusSignal.refreshAll(
				jointAbsolutePositionSignal,
				intakeVelocitySignal,
				intakeMotorVoltageSignal,
				intakeStatorCurrentSignal,
				intakeSupplyVoltageSignal,
				intakeSupplyCurrentSignal);

		inputs.jointPosRadians = jointAbsolutePositionSignal.getValueAsDouble() * 2 * PI;
		inputs.jointPosRadians = angleModulus(inputs.jointPosRadians - JOINT_ENCODER_ZERO);
		jointEncoder.setPosition(inputs.jointPosRadians);

		inputs.jointVelRadiansPerSec = jointEncoder.getVelocity();
		inputs.jointVoltageVolts = jointSparkMax.getBusVoltage() * jointSparkMax.getAppliedOutput();
		inputs.jointCurrentAmps = jointSparkMax.getOutputCurrent();
		inputs.jointBusVoltageVolts = jointSparkMax.getBusVoltage();

		inputs.intakeVelocityRPM = intakeVelocitySignal.getValueAsDouble() * 60.0;
		inputs.intakeVoltageVolts = intakeMotorVoltageSignal.getValueAsDouble();
		inputs.intakeCurrentAmps = intakeStatorCurrentSignal.getValueAsDouble();
		inputs.intakeBusVoltageVolts = intakeSupplyVoltageSignal.getValueAsDouble();
		inputs.intakeBusCurrentAmps = intakeSupplyCurrentSignal.getValueAsDouble();
	}

	public void actuate(IntakeInputs inputs, double jointVoltage, double intakeVoltage) {
		Logger.recordOutput("/Intake/jointCommandVoltageVolts", jointVoltage);

		if (!Robot.isReal()) return;

		jointSparkMax.setVoltage(jointVoltage);
		intakeTalonFX.setControl(intakeControlRequest.withOutput(intakeVoltage));
	}
}
