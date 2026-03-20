package frc.robot.subsystem.shooter;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.shooter.ShooterStartup;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
	public ShooterHW hw = new ShooterHW();
	public ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

	public static final double TURRET_POS_MIN = -2.029; // TUNEME
	// public static final double TURRET_POS_MAX = 1.087;
	public static final double TURRET_POS_MAX = PI / 2;
	public static final double TURRET_SNAP_THRES = 60 / 180.0 * PI;
	public static final double HOOD_POS_MIN = 0; //  TUNEME
	// FIXME MATCH HOOD POS WITH REAL INCLINE
	public static final double HOOD_POS_MAX = 35 / 180.0 * PI;

	public static final double FLYWHEEL_THRES = 100;
	public static final double HOOD_POS_THRES = 2 / 180.0 * PI;
	public static final double TURRET_POS_THRES = 2 / 180.0 * PI;

	public static final Constraints TURRET_CONSTRAINTS = new Constraints(6.0, 240.0); // TUNEME
	public static TrapezoidProfile turretProfile = new TrapezoidProfile(TURRET_CONSTRAINTS);
	public State turretSetpoint = new State();

	public static final Constraints HOOD_CONSTRAINTS = new Constraints(2.0, 16.0); // TUNEME
	public static TrapezoidProfile hoodProfile = new TrapezoidProfile(HOOD_CONSTRAINTS);
	public State hoodSetpoint = new State();

	public double targetTurretPos = 0;
	public double targetHoodPos = 0;
	public double targetFlywheelRPM = 0;
	public boolean onTarget = false;

	public boolean zeroed = false;

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
		if (!zeroed && robot.isTeleopEnabled() && !(getCurrentCommand() instanceof ShooterStartup)) {
			CommandScheduler.getInstance().schedule(new ShooterStartup());
		}
	}

	public void actuate() {
		targetTurretPos = angleModulus(targetTurretPos);
		if (targetTurretPos < TURRET_POS_MIN || targetTurretPos > TURRET_POS_MAX) {
			double distToMin = abs(angleModulus(targetTurretPos - TURRET_POS_MIN));
			double distToMax = abs(angleModulus(targetTurretPos - TURRET_POS_MAX));
			double nearest = distToMin < distToMax ? TURRET_POS_MIN : TURRET_POS_MAX;
			if (min(distToMin, distToMax) < TURRET_SNAP_THRES) {
				State turretGoal = new State(nearest, 0);
				turretSetpoint = turretProfile.calculate(LOOP_PERIOD, turretSetpoint, turretGoal);
			} else {
				turretSetpoint = new State(inputs.turretPosRadians, 0);
			}
		} else {
			State turretGoal = new State(targetTurretPos, 0);
			turretSetpoint = turretProfile.calculate(LOOP_PERIOD, turretSetpoint, turretGoal);
		}

		targetHoodPos = clamp(targetHoodPos, HOOD_POS_MIN, HOOD_POS_MAX);
		State hoodGoal = new State(targetHoodPos, 0);
		hoodSetpoint = hoodProfile.calculate(LOOP_PERIOD, hoodSetpoint, hoodGoal);

		Logger.recordOutput("/Shooter/targetTurretPosRadians", targetTurretPos);
		Logger.recordOutput("/Shooter/turretProfilePosRadians", turretSetpoint.position);
		Logger.recordOutput("/Shooter/targetHoodPosRadians", targetHoodPos);
		Logger.recordOutput("/Shooter/hoodProfilePosRadians", hoodSetpoint.position);
		Logger.recordOutput("/Shooter/targetFlywheelRPM", targetFlywheelRPM);
		Logger.recordOutput("/Shooter/turretProfileVelRadiansPerSec", turretSetpoint.velocity);
		Logger.recordOutput("/Shooter/hoodProfileVelRadiansPerSec", hoodSetpoint.velocity);

		onTarget = abs(inputs.turretPosRadians - targetTurretPos) < TURRET_POS_THRES
				&& abs(inputs.hoodPosRadians - targetHoodPos) < HOOD_POS_THRES
				&& abs(inputs.flywheelVelocityRPM - targetFlywheelRPM) < FLYWHEEL_THRES;
		Logger.recordOutput("/Shooter/onTarget", onTarget);
		hw.actuate(inputs, turretSetpoint.position, hoodSetpoint.position, targetFlywheelRPM);
	}
}
