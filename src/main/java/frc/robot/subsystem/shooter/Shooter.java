package frc.robot.subsystem.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
	public ShooterHW hw = new ShooterHW();
	public ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

	public double targetTurretPos = 0;
	public double targetHoodPos = 0;
	public double targetFlywheelRPM = 0;

	// public static final Constraints TURRET_CONSTRAINTS = new Constraints(3.0, 48.0); // TODO
	public static final Constraints TURRET_CONSTRAINTS = new Constraints(1e9, 1e9);
	public static TrapezoidProfile turretProfile = new TrapezoidProfile(TURRET_CONSTRAINTS);
	public State turretSetpoint = new State();

	// public static final Constraints HOOD_CONSTRAINTS = new Constraints(2.0, 16.0);
	public static final Constraints HOOD_CONSTRAINTS = new Constraints(1e9, 1e9);
	public static TrapezoidProfile hoodProfile = new TrapezoidProfile(HOOD_CONSTRAINTS);
	public State hoodSetpoint = new State();

	public void init() {
		hw.init();
		sense();

		turretSetpoint = new State(inputs.turretPosRadians, inputs.turretVelRadiansPerSec);
		hoodSetpoint = new State(inputs.hoodPosRadians, inputs.hoodVelRadiansPerSec);
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Shooter", inputs);

		if (robot.isDisabled()) {
			targetHoodPos = inputs.hoodPosRadians;
			targetTurretPos = inputs.turretPosRadians;
		}
	}

	public void actuate() {
		State turretGoal = new State(targetTurretPos, 0);
		turretSetpoint = turretProfile.calculate(LOOP_PERIOD, turretSetpoint, turretGoal);

		State hoodGoal = new State(targetHoodPos, 0);
		hoodSetpoint = hoodProfile.calculate(LOOP_PERIOD, hoodSetpoint, hoodGoal);

		Logger.recordOutput("/Shooter/targetTurretPosRadians", targetTurretPos);
		Logger.recordOutput("/Shooter/turretProfilePosRadians", turretSetpoint.position);
		Logger.recordOutput("/Shooter/targetHoodPosRadians", targetHoodPos);
		Logger.recordOutput("/Shooter/hoodProfilePosRadians", hoodSetpoint.position);
		Logger.recordOutput("/Shooter/targetFlywheelRPM", targetFlywheelRPM);
		Logger.recordOutput("/Shooter/turretProfileVelRadiansPerSec", turretSetpoint.velocity);
		Logger.recordOutput("/Shooter/hoodProfileVelRadiansPerSec", hoodSetpoint.velocity);

		hw.actuate(inputs, turretSetpoint.position, hoodSetpoint.position, targetFlywheelRPM);
	}
}
