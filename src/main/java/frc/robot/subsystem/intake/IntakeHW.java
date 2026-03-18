package frc.robot.subsystem.intake;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.intake;
import static java.lang.Math.*;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
	public static final double EXTENSION_RATIO = 1.0; //TODO

	public static final double EXTENSION_ZERO = 0.0; //TODO

	public TalonFX extensionTalonFX;
	public TalonFX intakeTalonFX;

	public StatusSignal<Angle> extensionAbsolutePosition;
	public StatusSignal<AngularVelocity> extensionVelocity;
	public StatusSignal<Voltage> extensionMotorVoltage;
	public StatusSignal<Current> extensionStatorCurrent;
	public StatusSignal<Voltage> extensionSupplyVoltage;
	public StatusSignal<Current> extensionSupplyCurrent;

	public StatusSignal<AngularVelocity> intakeVelocity;
	public StatusSignal<Voltage> intakeMotorVoltage;
	public StatusSignal<Current> intakeStatorCurrent;
	public StatusSignal<Voltage> intakeSupplyVoltage;
	public StatusSignal<Current> intakeSupplyCurrent;

	public void init() {
		if (!Robot.isReal()) return;

		extensionTalonFX = new TalonFX(33); //TODO
		TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
		extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		extensionConfig.CurrentLimits.SupplyCurrentLimit = 30;
		extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		extensionConfig.CurrentLimits.StatorCurrentLimit = 120;
		extensionConfig.Voltage.PeakForwardVoltage = 12.0;
		extensionConfig.Voltage.PeakReverseVoltage = -12.0;
		extensionTalonFX.getConfigurator().apply(extensionConfig);
		extensionTalonFX.clearStickyFaults();
		extensionTalonFX.setNeutralMode(NeutralModeValue.Brake);
		extensionTalonFX.optimizeBusUtilization();

		intakeTalonFX = new TalonFX(31); //TODO
		TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
		intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.SupplyCurrentLimit = 30;
		intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.StatorCurrentLimit = 120;
		intakeConfig.Voltage.PeakForwardVoltage = 12.0;
		intakeConfig.Voltage.PeakReverseVoltage = -12.0;
		intakeTalonFX.getConfigurator().apply(intakeConfig);
		intakeTalonFX.clearStickyFaults();
		intakeTalonFX.setNeutralMode(NeutralModeValue.Brake);
		intakeTalonFX.optimizeBusUtilization();

		extensionAbsolutePosition = extensionTalonFX.getPosition(false);
		extensionVelocity = extensionTalonFX.getVelocity(false);
		extensionMotorVoltage = extensionTalonFX.getMotorVoltage(false);
		extensionStatorCurrent = extensionTalonFX.getStatorCurrent(false);
		extensionSupplyVoltage = extensionTalonFX.getSupplyVoltage(false);
		extensionSupplyCurrent = extensionTalonFX.getSupplyCurrent(false);

		intakeVelocity = intakeTalonFX.getVelocity(false);
		intakeMotorVoltage = intakeTalonFX.getMotorVoltage(false);
		intakeStatorCurrent = intakeTalonFX.getStatorCurrent(false);
		intakeSupplyVoltage = intakeTalonFX.getSupplyVoltage(false);
		intakeSupplyCurrent = intakeTalonFX.getSupplyCurrent(false);

		extensionAbsolutePosition.setUpdateFrequency(50);
		extensionVelocity.setUpdateFrequency(50);
		extensionMotorVoltage.setUpdateFrequency(50);
		extensionStatorCurrent.setUpdateFrequency(50);
		extensionSupplyVoltage.setUpdateFrequency(10);
		extensionSupplyCurrent.setUpdateFrequency(10);

		intakeVelocity.setUpdateFrequency(50);
		intakeMotorVoltage.setUpdateFrequency(50);
		intakeStatorCurrent.setUpdateFrequency(50);
		intakeSupplyVoltage.setUpdateFrequency(10);
		intakeSupplyCurrent.setUpdateFrequency(10);
	}

	public void sense(IntakeInputs inputs) {
		if (!Robot.isReal()) return;

		extensionAbsolutePosition.refresh();

		inputs.extensionPosMeters = extensionAbsolutePosition.getValueAsDouble() * 2 * Math.PI * EXTENSION_RATIO - EXTENSION_ZERO;
		inputs.extensionVelMetersPerSec = extensionVelocity.getValueAsDouble() * 2 * Math.PI * EXTENSION_RATIO;
		inputs.extensionVoltageVolts = extensionMotorVoltage.getValueAsDouble();
		inputs.extensionCurrentAmps = extensionStatorCurrent.getValueAsDouble();
		inputs.extensionBusVoltageVolts = extensionSupplyVoltage.getValueAsDouble();
		inputs.extensionBusCurrentAmps = extensionSupplyCurrent.getValueAsDouble();	

		inputs.intakeVelocityRPM = intakeVelocity.getValueAsDouble() * 60;
		inputs.intakeVoltageVolts = intakeMotorVoltage.getValueAsDouble();
		inputs.intakeCurrentAmps = intakeStatorCurrent.getValueAsDouble();
		inputs.intakeBusVoltageVolts = intakeSupplyVoltage.getValueAsDouble();
		inputs.intakeBusCurrentAmps = intakeSupplyCurrent.getValueAsDouble();
	}

	public void actuate(IntakeInputs inputs, double jointVoltage, double intakeVoltage) {
		Logger.recordOutput("/Intake/jointCommandVoltageVolts", jointVoltage);

		if (!Robot.isReal()) return;

		extensionTalonFX.setVoltage(jointVoltage);
		intakeTalonFX.setVoltage(intakeVoltage);
	}
}
