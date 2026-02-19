package frc.robot.subsystem.shooter;

import static java.lang.Math.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Robot;

public class ShooterHW {
	public static final double FLYWHEEL_RATIO = 1.0; // FIXME. motor * ratio = mechanism
	public static final double HOOD_RATIO = 1.0; // FIXME. motor * ratio = mechanism

	public static final double FLYWHEEL_kP = 0.0; // FIXME. V/(mechanism RPM)
	public static final double FLYWHEEL_kV = 0.0; // FIXME. V/(mechanism RPM)

	public static final double HOOD_kP = 0.5; // FIXME.
	public static final double HOOD_kD = 0.0; // FIXME.

	public TalonFX flyWheelMotor;
	public TalonFX flyWheel2Motor;
	public TalonFX hoodMotor;
	public TalonFX indexer;

	public VelocityVoltage flyWheelControlRequest = new VelocityVoltage(0).withSlot(0);
	public PositionVoltage hoodControlRequest = new PositionVoltage(0).withSlot(0);

	public void init() {
		if (!Robot.isReal()) return;

		flyWheelMotor = new TalonFX(0);
		TalonFXConfiguration flyWheelConfig = new TalonFXConfiguration();

		flyWheelConfig.Slot0.kP = FLYWHEEL_kP * FLYWHEEL_RATIO * 60;
		flyWheelConfig.Slot0.kV = FLYWHEEL_kV * FLYWHEEL_RATIO * 60;

		flyWheelMotor.getConfigurator().apply(flyWheelConfig);
		flyWheelMotor.clearStickyFaults();
		flyWheelMotor.setNeutralMode(NeutralModeValue.Coast);

		flyWheel2Motor = new TalonFX(2);
		flyWheel2Motor.setControl(new Follower(0, MotorAlignmentValue.Opposed));
		flyWheel2Motor.clearStickyFaults();

		hoodMotor = new TalonFX(1);
		TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

		hoodConfig.Slot0.kP = HOOD_kP * HOOD_RATIO * 2 * PI;
		hoodConfig.Slot0.kD = HOOD_kD * HOOD_RATIO * 2 * PI;

		hoodMotor.getConfigurator().apply(hoodConfig);
		hoodMotor.clearStickyFaults();
		hoodMotor.setNeutralMode(NeutralModeValue.Brake);

		indexer = new TalonFX(3);
		indexer.clearStickyFaults();
	}

	public void sense(ShooterInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.flyWheelVelocityRPM = flyWheelMotor.getVelocity().getValueAsDouble() * 60 * FLYWHEEL_RATIO;
		inputs.flyWheelVoltage = flyWheelMotor.getMotorVoltage().getValueAsDouble();
		inputs.flyWheelCurrent = flyWheelMotor.getStatorCurrent().getValueAsDouble();

		inputs.hoodAngleRad = hoodMotor.getPosition().getValueAsDouble() * 2 * PI * HOOD_RATIO;
		inputs.hoodVoltage = hoodMotor.getMotorVoltage().getValueAsDouble();
		inputs.hoodCurrent = hoodMotor.getStatorCurrent().getValueAsDouble();

		inputs.indexerVoltage = indexer.getMotorVoltage().getValueAsDouble();
		inputs.indexerCurrent = indexer.getStatorCurrent().getValueAsDouble();
	}

	public void setFlyWheelVelocity(double rpm) {
		if (!Robot.isReal()) return;
		flyWheelMotor.setControl(flyWheelControlRequest.withVelocity(rpm / 60 / FLYWHEEL_RATIO));
	}

	public void setFlyWheelVoltage(double voltage) {
		if (!Robot.isReal()) return;
		flyWheelMotor.setVoltage(voltage);
	}

	public void setHoodAngle(double angleRad) {
		if (!Robot.isReal()) return;
		hoodMotor.setControl(hoodControlRequest.withPosition(angleRad / (HOOD_RATIO * 2 * PI)));
	}

	public void setIndexerVoltage(double voltage) {
		if (!Robot.isReal()) return;
		indexer.setVoltage(voltage);
	}
}
