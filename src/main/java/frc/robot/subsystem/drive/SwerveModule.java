package frc.robot.subsystem.drive;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {

	public static double DRIVE_kS = 0.093052;
	public static double DRIVE_kV = 1.8968;
	public static double DRIVE_kA = 0.15096;
	public static SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(DRIVE_kS, DRIVE_kV, DRIVE_kA);

	public static double TURN_kS = 0.19431;
	public static double TURN_kV = 0.36606;
	public static double TURN_kA = 0.044138;
	public static SimpleMotorFeedforward TURN_FF = new SimpleMotorFeedforward(TURN_kS, TURN_kV, TURN_kA);

	public static double MAX_VEL = 6.3;

	public static TrapezoidProfile.Constraints TURN_CONSTRAINTS = new TrapezoidProfile.Constraints(21, 250);
	public static TrapezoidProfile profile = new TrapezoidProfile(TURN_CONSTRAINTS);

	public SwerveHW hw = new SwerveHW();
	public SwerveInputsAutoLogged inputs;
	public String name;

	public void init(SwerveModuleConfiguration config) {
		inputs = new SwerveInputsAutoLogged();
		hw.init(config);
		hw.sense(inputs);
		name = config.name;
		setpointState = new TrapezoidProfile.State(inputs.turnPos, inputs.turnVel);
	}

	public SwerveModuleState currentState = new SwerveModuleState();
	public double currentAngle;
	public double currentVel;
	public double currentDrivePos;

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs(hw.logRoot, inputs);

		currentState = new SwerveModuleState(inputs.driveVel, new Rotation2d(inputs.turnPos));
		currentAngle = inputs.turnPos;
		currentVel = inputs.driveVel;
		currentDrivePos = inputs.drivePos;
	}

	public SwerveModuleState targetState = new SwerveModuleState();
	public TrapezoidProfile.State setpointState = new TrapezoidProfile.State();
	public double setpointVel = 0;

	public void drive(SwerveModuleState state) {
		targetState = state;
		state.optimize(new Rotation2d(inputs.turnAbsPos));
	}

	public void periodic() {
		sense();

		double targetAngle = targetState.angle.getRadians();
		targetAngle = setpointState.position + angleModulus(targetAngle - setpointState.position);
		double targetVel = targetState.speedMetersPerSecond * cos(currentAngle - targetAngle);

		TrapezoidProfile.State newSetpointState =
				profile.calculate(LOOP_PERIOD, setpointState, new TrapezoidProfile.State(targetAngle, 0));
		double turnFF = TURN_FF.calculateWithVelocities(setpointState.velocity, newSetpointState.velocity);
		setpointState = newSetpointState;

		double driveFF = DRIVE_FF.calculateWithVelocities(setpointVel, targetVel);
		setpointVel = targetVel;

		hw.actuate(inputs, targetVel, driveFF, setpointState.position, turnFF);

		Logger.recordOutput(hw.logRoot + "/targetAngle", angleModulus(targetAngle));
		Logger.recordOutput(hw.logRoot + "/targetVel", targetVel);
		Logger.recordOutput(hw.logRoot + "/targetVelCos", targetVel);
		Logger.recordOutput(hw.logRoot + "/turnProfiledPos", angleModulus(setpointState.position));
		Logger.recordOutput(hw.logRoot + "/turnProfiledVel", setpointState.velocity);
		Logger.recordOutput(hw.logRoot + "/driveFF", driveFF);
		Logger.recordOutput(hw.logRoot + "/turnFF", turnFF);
	}
}
