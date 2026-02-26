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

	public double targetTurretPos = 0; // radians
	public double targetHoodPos = 0; // radians
	public double targetFlywheelVel = 0; // rad/s

	// Turret trapezoid profile
	public static final Constraints TURRET_CONSTRAINTS = new Constraints(3.0, 6.0); // FIXME rad/s, rad/s^2
	public static TrapezoidProfile turretProfile = new TrapezoidProfile(TURRET_CONSTRAINTS);
	public State turretSetpoint = new State();

	// Hood trapezoid profile
	public static final Constraints HOOD_CONSTRAINTS = new Constraints(2.0, 4.0); // FIXME
	public static TrapezoidProfile hoodProfile = new TrapezoidProfile(HOOD_CONSTRAINTS);
	public State hoodSetpoint = new State();

	public void init() {
		hw.init();
		sense();

		turretSetpoint = new State(inputs.turretPos, inputs.turretVel);
		hoodSetpoint = new State(inputs.hoodPos, inputs.hoodVel);
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Shooter", inputs);
	}

	public void actuate() {
		State turretGoal = new State(targetTurretPos, 0);
		turretSetpoint = turretProfile.calculate(LOOP_PERIOD, turretSetpoint, turretGoal);

		State hoodGoal = new State(targetHoodPos, 0);
		hoodSetpoint = hoodProfile.calculate(LOOP_PERIOD, hoodSetpoint, hoodGoal);

		Logger.recordOutput("/Shooter/targetTurretPos", targetTurretPos);
		Logger.recordOutput("/Shooter/turretProfilePos", turretSetpoint.position);
		Logger.recordOutput("/Shooter/targetHoodPos", targetHoodPos);
		Logger.recordOutput("/Shooter/hoodProfilePos", hoodSetpoint.position);
		Logger.recordOutput("/Shooter/targetFlywheelVel", targetFlywheelVel);

		hw.actuate(inputs, turretSetpoint.position, hoodSetpoint.position, targetFlywheelVel);
	}
}
