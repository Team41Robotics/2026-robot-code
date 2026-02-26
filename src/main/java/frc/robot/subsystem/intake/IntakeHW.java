package frc.robot.subsystem.intake;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class IntakeHW {
	public static double JOINT_RATIO = 0.2; // FIXME. joint gear ratio (motor * ratio = mechanism)
	public static double JOINT_kP = 10.0; // FIXME. joint PID P
	public static double JOINT_kD = 0.0; // FIXME. joint PID D

	public TalonFX jointTalonFX;
	public TalonFX intakeTalonFX;

	public PositionVoltage jointControlRequest = new PositionVoltage(0).withSlot(0);
	public VoltageOut intakeControlRequest = new VoltageOut(0);

	public void init() {
		if (!Robot.isReal()) return;

		jointTalonFX = new TalonFX(31);
		TalonFXConfiguration jointConfig = new TalonFXConfiguration();

		jointConfig.Slot0.kP = JOINT_kP * JOINT_RATIO * 2 * PI;
		jointConfig.Slot0.kD = JOINT_kD * JOINT_RATIO * 2 * PI;

		jointConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		jointConfig.CurrentLimits.SupplyCurrentLimit = 40; // FIXME. supply current limit (A)
		jointConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		jointConfig.CurrentLimits.StatorCurrentLimit = 60; // FIXME. stator current limit (A)

		jointConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // FIXME. inversion

		jointTalonFX.getConfigurator().apply(jointConfig);
		jointTalonFX.clearStickyFaults();
		jointTalonFX.setPosition(0);
		jointTalonFX.setNeutralMode(NeutralModeValue.Brake);

		intakeTalonFX = new TalonFX(32);
		TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

		intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.SupplyCurrentLimit = 40; // FIXME. supply current limit (A)
		intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.StatorCurrentLimit = 60; // FIXME. stator current limit (A)

		intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // FIXME. inversion

		intakeTalonFX.getConfigurator().apply(intakeConfig);
		intakeTalonFX.clearStickyFaults();
		intakeTalonFX.setNeutralMode(NeutralModeValue.Coast);
	}

	public void sense(IntakeInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.jointPos = jointTalonFX.getPosition().getValueAsDouble() * JOINT_RATIO * 2 * PI;
		inputs.jointVel = jointTalonFX.getVelocity().getValueAsDouble() * JOINT_RATIO * 2 * PI;
		inputs.jointVoltage = jointTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.jointCurrent = jointTalonFX.getStatorCurrent().getValueAsDouble();
		inputs.jointBusVoltage = jointTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.jointBusCurrent = jointTalonFX.getSupplyCurrent().getValueAsDouble();

		inputs.intakeVoltage = intakeTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.intakeCurrent = intakeTalonFX.getStatorCurrent().getValueAsDouble();
		inputs.intakeBusVoltage = intakeTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.intakeBusCurrent = intakeTalonFX.getSupplyCurrent().getValueAsDouble();
	}

	public void actuate(IntakeInputs inputs, double jointPosition, double intakeVoltage) {
		Logger.recordOutput("/Intake/actuatedJointPos", jointPosition);
		Logger.recordOutput("/Intake/actuatedIntakeVoltage", intakeVoltage);

		if (!Robot.isReal()) return;

		// jointTalonFX.setControl(jointControlRequest.withPosition(jointPosition / (JOINT_RATIO * 2 * PI)));
		// intakeTalonFX.setControl(intakeControlRequest.withOutput(intakeVoltage));
		// intakeTalonFX.setVoltage(-12);
		jointTalonFX.setVoltage(12);
	}

	public void zeroJointPosition() {
		if (!Robot.isReal()) return;
		jointTalonFX.setPosition(0);
	}
}
