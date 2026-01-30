package frc.robot.subsystem.drive;

import static java.lang.Math.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
	public static SimpleMotorFeedforward TURN_FF = new SimpleMotorFeedforward(TURN_kS, TURN_kV, TURN_kA);

	public static double MAX_VEL = 0;

	public static TrapezoidProfile.Constraints TURN_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
	public static TrapezoidProfile profile = new TrapezoidProfile(TURN_CONSTRAINTS);

	public static double VELOCITY_DEADBAND = 0.01;

	public SwerveHW hw;
	public String name;

	public void init(SwerveModuleConfiguration config) {
		hw.init(config);
		hw.sense();
		name = config.name;
		setpointState = new TrapezoidProfile.State(hw.turnPos, hw.turnVel);
	}

	public SwerveModuleState currentState;
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

	public SwerveModuleState targetState;
	public TrapezoidProfile.State setpointState;

	public void drive(SwerveModuleState state) {
		targetState = state;
	}

	public void periodic() {
		sense();

		double targetAngle = targetState.angle.getRadians();
		double targetVel = targetState.speedMetersPerSecond * cos(currentAngle - targetAngle);
		if (abs(targetVel) < VELOCITY_DEADBAND) {
			targetVel = 0;
			targetAngle = currentAngle;
		}

		double driveFF = DRIVE_kS * signum(targetVel) + DRIVE_kV * targetVel;

		TrapezoidProfile.State newSetpointState =
				profile.calculate(Robot.kDefaultPeriod, setpointState, new TrapezoidProfile.State(targetAngle, 0));
		double turnFF = TURN_FF.calculateWithVelocities(setpointState.velocity, newSetpointState.velocity);
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
