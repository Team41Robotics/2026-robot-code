package frc.robot.subsystem.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;

public class ClimberHW {
	public static final double SPOOL_DIAMETER_METERS = 0.025; // TUNEME: spool diameter in meters
	public static final double GEAR_RATIO = 1.0 / 10.0; // TUNEME: motor * ratio = mechanism
	public static final double METERS_PER_ROTATION = GEAR_RATIO * Math.PI * SPOOL_DIAMETER_METERS;

	public TalonFX leader;
	public TalonFX follower;

	public DigitalInput limitSwitchTop;
	public DigitalInput limitSwitchTop2;
	public DigitalInput limitSwitchBottom;
	public DigitalInput limitSwitchBottom2;

	public SparkMax actuator;
	public DigitalInput limitSwitchActuator;

	// Cached StatusSignals
	public StatusSignal<Angle> position;
	public StatusSignal<AngularVelocity> velocity;
	public StatusSignal<Voltage> motorVoltage;
	public StatusSignal<Current> statorCurrent;

	public void init() {
		leader = new TalonFX(60);
		follower = new TalonFX(61);

		limitSwitchTop = new DigitalInput(2); // TODO: DIO port
		limitSwitchBottom = new DigitalInput(3); // TODO: DIO port
		limitSwitchTop2 = new DigitalInput(4); // TODO: DIO port
		limitSwitchBottom2 = new DigitalInput(5); // TODO: DIO port

		leader.setNeutralMode(NeutralModeValue.Brake);
		follower.setNeutralMode(NeutralModeValue.Brake);
		follower.setControl(new Follower(60, MotorAlignmentValue.Aligned));

		actuator = new SparkMax(9, MotorType.kBrushed);
		SparkMaxConfig actuatorConfig = new SparkMaxConfig();
		actuatorConfig.smartCurrentLimit(60);
		actuatorConfig.idleMode(IdleMode.kBrake);
		actuatorConfig.inverted(true);
		actuator.configure(actuatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		limitSwitchActuator = new DigitalInput(6); // TODO: DIO port

		// Initialize cached signals
		position = leader.getPosition(false);
		velocity = leader.getVelocity(false);
		motorVoltage = leader.getMotorVoltage(false);
		statorCurrent = leader.getStatorCurrent(false);

		// Set update frequencies
		position.setUpdateFrequency(50);
		velocity.setUpdateFrequency(50);
		motorVoltage.setUpdateFrequency(50);
		statorCurrent.setUpdateFrequency(50);

		// Keep signals alive that the follower needs from the leader
		leader.getDutyCycle(false).setUpdateFrequency(50);
		leader.getTorqueCurrent(false).setUpdateFrequency(50);

		// Optimize bus utilization
		leader.optimizeBusUtilization();
		follower.optimizeBusUtilization();
	}

	public void sense(ClimberInputs inputs) {
		BaseStatusSignal.waitForAll(0, position, velocity, motorVoltage, statorCurrent);

		inputs.posMeters = position.getValueAsDouble() * METERS_PER_ROTATION;
		inputs.velMetersPerSec = velocity.getValueAsDouble() * METERS_PER_ROTATION;
		inputs.voltageVolts = motorVoltage.getValueAsDouble();
		inputs.currentAmps = statorCurrent.getValueAsDouble();

		inputs.limitTop = !limitSwitchTop.get();
		inputs.limitTop2 = !limitSwitchTop2.get();
		inputs.limitBottom = !limitSwitchBottom.get();
		inputs.limitBottom2 = !limitSwitchBottom2.get();

		inputs.actuatorVoltageVolts = actuator.getBusVoltage() * actuator.getAppliedOutput();
		inputs.actuatorCurrentAmps = actuator.getOutputCurrent();
		inputs.limitActuator = !limitSwitchActuator.get();
	}

	public void actuate(double targetVoltage, double actuatorTargetVoltage) {
		leader.setVoltage(targetVoltage);
		actuator.setVoltage(actuatorTargetVoltage);
		Logger.recordOutput("Climber/targetVoltage", targetVoltage);
		Logger.recordOutput("Climber/actuatorTargetVoltage", actuatorTargetVoltage);
	}
}
