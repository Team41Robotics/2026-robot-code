package frc.robot.subsystem.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Robot;

public class ShooterHW {
	public static final double SHOOTER_RATIO = 1.0; // FIXME. motor rotations * ratio = mechanism RPM

	public static final double SHOOTER_kP = 6e-3 / 12.0; // duty_cycle/RPM
	public static final double SHOOTER_kD = 2.5e-4 / 12.0; // duty_cycle/(RPM/s)
	public static final double SHOOTER_kV = (0.0019203 / 1.07) / 12.0; // duty_cycle/RPM
	public static final double SHOOTER_kS = (0.0019203 / 1.07 * 30) / 12.0; // duty_cycle

	public SparkFlex shooterMotor;
	public SparkFlex shooterFollower;
	public SparkClosedLoopController shooterController;

	public void init() {
		if (!Robot.isReal()) return;

		shooterMotor = new SparkFlex(41, MotorType.kBrushless);
		SparkFlexConfig shooterConfig = new SparkFlexConfig();
		shooterConfig.closedLoop.pid(SHOOTER_kP, 0, SHOOTER_kD);
		shooterConfig.closedLoop.feedForward.kS(SHOOTER_kS).kV(SHOOTER_kV);
		shooterConfig.idleMode(IdleMode.kCoast);
		shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		shooterMotor.clearFaults();
		shooterController = shooterMotor.getClosedLoopController();

		shooterFollower = new SparkFlex(26, MotorType.kBrushless);
		SparkFlexConfig followerConfig = new SparkFlexConfig().apply(shooterConfig);
		followerConfig.follow(shooterMotor, true);
		shooterFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		shooterFollower.clearFaults();
	}

	public void sense(ShooterInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.velocityRPM = shooterMotor.getEncoder().getVelocity();
		inputs.voltage = shooterMotor.getAppliedOutput() * shooterMotor.getBusVoltage();
		inputs.current = shooterMotor.getOutputCurrent();
	}

	public void setShooterVelocity(double rpm) {
		if (!Robot.isReal()) return;
		shooterController.setSetpoint(rpm / SHOOTER_RATIO, ControlType.kVelocity);
	}

	public void setShooterVoltage(double voltage) {
		if (!Robot.isReal()) return;
		shooterMotor.setVoltage(voltage);
	}
}
