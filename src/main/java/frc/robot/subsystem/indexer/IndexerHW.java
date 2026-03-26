package frc.robot.subsystem.indexer;

import static frc.robot.RobotContainer.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class IndexerHW {
	// Spark heartbeat tuning (competition-safe): poll 5–20 Hz, timeout 100–200 ms
	public static final double SPARK_HEARTBEAT_PERIOD_Sec = 0.10;

	public TalonFX spinTalonFX;
	public TalonFX elevatorTalonFX;
	public SparkFlex backvatorSparkFlex;

	private double backvatorLastHeartbeatPollTimeSec = Double.NEGATIVE_INFINITY;
	private double backvatorLastGoodTimeSec = Double.NEGATIVE_INFINITY;
	private int backvatorLastErrorCode = 0;

	public VoltageOut spinControlRequest = new VoltageOut(0);
	public VoltageOut elevatorControlRequest = new VoltageOut(0);

	public StatusSignal<AngularVelocity> spinVelocity;
	public StatusSignal<Voltage> spinMotorVoltage;
	public StatusSignal<Current> spinStatorCurrent;
	public StatusSignal<Voltage> spinSupplyVoltage;
	public StatusSignal<Current> spinSupplyCurrent;

	public StatusSignal<AngularVelocity> elevatorVelocity;
	public StatusSignal<Voltage> elevatorMotorVoltage;
	public StatusSignal<Current> elevatorStatorCurrent;
	public StatusSignal<Voltage> elevatorSupplyVoltage;
	public StatusSignal<Current> elevatorSupplyCurrent;

	public void init() {
		if (!Robot.isReal()) return;

		double now = Timer.getTimestamp();
		backvatorLastHeartbeatPollTimeSec = now;
		backvatorLastGoodTimeSec = now;

		spinTalonFX = new TalonFX(43, driveBus);
		TalonFXConfiguration spinConfig = new TalonFXConfiguration();
		spinConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		spinConfig.CurrentLimits.SupplyCurrentLimit = 30;
		spinConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		spinConfig.CurrentLimits.StatorCurrentLimit = 120;
		spinConfig.Voltage.PeakForwardVoltage = 12.0;
		spinConfig.Voltage.PeakReverseVoltage = -12.0;
		spinConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		spinTalonFX.getConfigurator().apply(spinConfig);
		spinTalonFX.clearStickyFaults();
		spinTalonFX.setNeutralMode(NeutralModeValue.Brake);

		spinVelocity = spinTalonFX.getVelocity(false);
		spinMotorVoltage = spinTalonFX.getMotorVoltage(false);
		spinStatorCurrent = spinTalonFX.getStatorCurrent(false);
		spinSupplyVoltage = spinTalonFX.getSupplyVoltage(false);
		spinSupplyCurrent = spinTalonFX.getSupplyCurrent(false);

		spinVelocity.setUpdateFrequency(50);
		spinMotorVoltage.setUpdateFrequency(50);
		spinStatorCurrent.setUpdateFrequency(50);
		spinSupplyVoltage.setUpdateFrequency(50);
		spinSupplyCurrent.setUpdateFrequency(50);

		spinTalonFX.optimizeBusUtilization();

		elevatorTalonFX = new TalonFX(41);
		TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
		elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		elevatorConfig.CurrentLimits.SupplyCurrentLimit = 20;
		elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		elevatorConfig.CurrentLimits.StatorCurrentLimit = 80;
		elevatorConfig.Voltage.PeakForwardVoltage = 12.0;
		elevatorConfig.Voltage.PeakReverseVoltage = -12.0;
		elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		elevatorTalonFX.getConfigurator().apply(elevatorConfig);
		elevatorTalonFX.clearStickyFaults();
		elevatorTalonFX.setNeutralMode(NeutralModeValue.Brake);

		elevatorVelocity = elevatorTalonFX.getVelocity(false);
		elevatorMotorVoltage = elevatorTalonFX.getMotorVoltage(false);
		elevatorStatorCurrent = elevatorTalonFX.getStatorCurrent(false);
		elevatorSupplyVoltage = elevatorTalonFX.getSupplyVoltage(false);
		elevatorSupplyCurrent = elevatorTalonFX.getSupplyCurrent(false);

		elevatorVelocity.setUpdateFrequency(50);
		elevatorMotorVoltage.setUpdateFrequency(50);
		elevatorStatorCurrent.setUpdateFrequency(50);
		elevatorSupplyVoltage.setUpdateFrequency(10);
		elevatorSupplyCurrent.setUpdateFrequency(10);

		elevatorTalonFX.optimizeBusUtilization();

		backvatorSparkFlex = new SparkFlex(44, MotorType.kBrushless);
		SparkFlexConfig backvatorConfig = new SparkFlexConfig();
		backvatorConfig.smartCurrentLimit(70);
		backvatorConfig.idleMode(IdleMode.kBrake);
		backvatorConfig.inverted(false);
		backvatorSparkFlex.configure(backvatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	public void sense(IndexerInputs inputs) {
		if (!Robot.isReal()) return;

		double now = Timer.getTimestamp();
		if ((now - backvatorLastHeartbeatPollTimeSec) >= SPARK_HEARTBEAT_PERIOD_Sec) {
			backvatorLastHeartbeatPollTimeSec = now;
			backvatorSparkFlex.getMotorTemperature();
			REVLibError err = backvatorSparkFlex.getLastError();
			backvatorLastErrorCode = err.value;
			if (err == REVLibError.kOk) {
				backvatorLastGoodTimeSec = now;
			}
		}

		BaseStatusSignal.waitForAll(
				0, spinVelocity, spinMotorVoltage, spinStatorCurrent, spinSupplyVoltage, spinSupplyCurrent);
		BaseStatusSignal.refreshAll(
				elevatorVelocity,
				elevatorMotorVoltage,
				elevatorStatorCurrent,
				elevatorSupplyVoltage,
				elevatorSupplyCurrent);

		inputs.spinVelocityRPM = spinVelocity.getValueAsDouble() * 60.0;
		inputs.spinVoltageVolts = spinMotorVoltage.getValueAsDouble();
		inputs.spinCurrentAmps = spinStatorCurrent.getValueAsDouble();
		inputs.spinBusVoltageVolts = spinSupplyVoltage.getValueAsDouble();
		inputs.spinBusCurrentAmps = spinSupplyCurrent.getValueAsDouble();
		inputs.spinTsSec = spinVelocity.getTimestamp().getTime();

		inputs.elevatorVelocityRPM = elevatorVelocity.getValueAsDouble() * 60.0;
		inputs.elevatorVoltageVolts = elevatorMotorVoltage.getValueAsDouble();
		inputs.elevatorCurrentAmps = elevatorStatorCurrent.getValueAsDouble();
		inputs.elevatorBusVoltageVolts = elevatorSupplyVoltage.getValueAsDouble();
		inputs.elevatorBusCurrentAmps = elevatorSupplyCurrent.getValueAsDouble();
		inputs.elevatorTsSec = elevatorVelocity.getTimestamp().getTime();

		inputs.backvatorVoltageVolts = backvatorSparkFlex.getBusVoltage() * backvatorSparkFlex.getAppliedOutput();
		inputs.backvatorCurrentAmps = backvatorSparkFlex.getOutputCurrent();
		inputs.backvatorVelocityRPM = backvatorSparkFlex.getEncoder().getVelocity();
		inputs.backvatorLastGoodTimeSec = backvatorLastGoodTimeSec;
		inputs.backvatorLastHeartbeatPollTimeSec = backvatorLastHeartbeatPollTimeSec;
		inputs.backvatorLastErrorCode = backvatorLastErrorCode;
	}

	public void actuate(IndexerInputs inputs, double spinVoltage, double elevatorVoltage, double backvatorVoltage) {
		if (!Robot.isReal()) return;

		spinTalonFX.setControl(spinControlRequest.withOutput(spinVoltage));
		elevatorTalonFX.setControl(elevatorControlRequest.withOutput(elevatorVoltage));
		backvatorSparkFlex.setVoltage(backvatorVoltage);
	}
}
