package frc.robot.subsystem.shooter;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.shooter.ShooterStartup;
import frc.robot.commands.shooter.Targetting;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
	public ShooterHW hw = new ShooterHW();
	public ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

	public static final double TURRET_POS_MIN = -2.029; // TUNEME
	// public static final double TURRET_POS_MAX = 1.087;
	public static final double TURRET_POS_MAX = PI / 2;
	public static final double TURRET_SNAP_THRES = 60 / 180.0 * PI;
	public static final double HOOD_POS_MIN = 0; //  TUNEME
	public static final double HOOD_POS_MAX = 35 / 180.0 * PI;

	public static final double FLYWHEEL_THRES = 200;
	public static final double HOOD_POS_THRES = 3 / 180.0 * PI;
	public static final double TURRET_POS_THRES = 3 / 180.0 * PI;

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

		// --- Turret field visualization ---
		Pose2d turretFieldPose = drive.pose.plus(Targetting.TURRET_POS);
		Translation2d turretPos = turretFieldPose.getTranslation();
		double robotAngle = drive.rot.getRadians();

		// Current direction arrow
		double turretFieldAngle = robotAngle + inputs.turretPosRadians;
		Logger.recordOutput("/Shooter/turretPose", new Pose2d(turretPos, new Rotation2d(turretFieldAngle)));

		// Target direction arrow
		double targetTurretFieldAngle = robotAngle + targetTurretPos;
		Logger.recordOutput("/Shooter/targetTurretPose", new Pose2d(turretPos, new Rotation2d(targetTurretFieldAngle)));

		// Wedge outline: center -> min ray arc -> max ray -> center
		double WEDGE_RANGE = 3.0; // meters, visual only
		int arcPoints = 20;
		Pose2d[] wedge = new Pose2d[arcPoints + 3];

		wedge[0] = new Pose2d(turretPos, new Rotation2d(robotAngle + TURRET_POS_MIN));
		for (int i = 0; i <= arcPoints; i++) {
			double frac = (double) i / arcPoints;
			double angle = TURRET_POS_MIN + frac * (TURRET_POS_MAX - TURRET_POS_MIN);
			double fieldAngle = robotAngle + angle;
			Translation2d point = turretPos.plus(new Translation2d(WEDGE_RANGE, new Rotation2d(fieldAngle)));
			wedge[i + 1] = new Pose2d(point, new Rotation2d(fieldAngle));
		}
		wedge[arcPoints + 2] = new Pose2d(turretPos, new Rotation2d(robotAngle + TURRET_POS_MAX));

		Logger.recordOutput("/Shooter/turretRange", wedge);

		hw.actuate(inputs, turretSetpoint.position, hoodSetpoint.position, targetFlywheelRPM);
	}
}
