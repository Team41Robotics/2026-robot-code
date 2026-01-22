package frc.robot.subsystem.drive;

import static java.lang.Math.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {

	public static double DRIVE_kS = 0;
	public static double DRIVE_kV = 0;

	public static double TURN_kS = 0;
	public static double TURN_kV = 0;
	public static double TURN_kA = 0;

	public static double MAX_VEL = 0;

	public static TrapezoidProfile.Constraints TURN_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
	public static TrapezoidProfile profile = new TrapezoidProfile(TURN_CONSTRAINTS);

	public static double VELOCITY_DEADBAND = 0.01;

	public SwerveHW hw = new SwerveHW();
	public String name;

	public void init(SwerveModuleConfiguration config) {
		hw.init(config);
		hw.sense();
		name = config.name;
		setpointState = new TrapezoidProfile.State(hw.turnPos, hw.turnVel);
	}

	public SwerveModuleState currentState = new SwerveModuleState();
	public double currentAngle;
	public double currentVel;
	public double currentDrivePos;

	public void sense() {
		hw.sense();

		currentState = new SwerveModuleState(hw.driveVel, new Rotation2d(hw.turnPos));
		currentAngle = hw.turnPos;
		currentVel = hw.driveVel;
		currentDrivePos = hw.drivePos;
	}

	public SwerveModuleState targetState = new SwerveModuleState();
	public TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

	public void drive(SwerveModuleState state) {
		targetState = state;
	}

	public void periodic() {
		sense();

		double targetAngle = targetState.angle.getRadians();
		double targetVel = targetState.speedMetersPerSecond * cos(currentAngle - targetAngle);
		targetVel = MathUtil.applyDeadband(targetVel, VELOCITY_DEADBAND);

		double driveFF = DRIVE_kS * signum(targetVel) + DRIVE_kV * targetVel;

		TrapezoidProfile.State newSetpointState =
				profile.calculate(Robot.kDefaultPeriod, setpointState, new TrapezoidProfile.State(targetAngle, 0));
		double turnFF = TURN_kS * signum(newSetpointState.velocity)
				+ TURN_kV * newSetpointState.velocity
				+ TURN_kA * (newSetpointState.velocity - setpointState.velocity) / Robot.kDefaultPeriod;
		setpointState = newSetpointState;

		hw.actuate(targetVel, driveFF, targetAngle, turnFF);

		Logger.recordOutput(hw.logRoot + "/targetAngle", targetAngle);
		Logger.recordOutput(hw.logRoot + "/targetVel", targetVel);
		Logger.recordOutput(hw.logRoot + "/targetVelCos", targetVel);
		Logger.recordOutput(hw.logRoot + "/turnProfiledPos", setpointState.position);
		Logger.recordOutput(hw.logRoot + "/turnProfiledVel", setpointState.velocity);
		Logger.recordOutput(hw.logRoot + "/driveFF", driveFF);
		Logger.recordOutput(hw.logRoot + "/turnFF", turnFF);
	}
}
