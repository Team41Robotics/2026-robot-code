package frc.robot.subsystem.drive;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
	public static double DRIVE_kS = 0.093052; // FIXME. feedforward ks (tune)
	public static double DRIVE_kV = 1.8968; // FIXME. feedforward kv (tune)
	public static double DRIVE_kA = 0.15096; // FIXME. feedforward ka (tune)
	public static SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(DRIVE_kS, DRIVE_kV, DRIVE_kA);

	public static double TURN_kS = 0.19431; // FIXME. turn feedforward ks (tune)
	public static double TURN_kV = 0.36606; // FIXME. turn feedforward kv (tune)
	// public static double TURN_kA = 0.044138;
	public static double TURN_kA = 0.; // FIXME. turn feedforward ka (tune)
	public static SimpleMotorFeedforward TURN_FF = new SimpleMotorFeedforward(TURN_kS, TURN_kV, TURN_kA);

	public static double MAX_VEL = 6.0; // FIXME. max wheel velocity (m/s)

	public static Constraints DRIVE_CONSTRAINTS = new Constraints(45, 1e9); // FIXME. drive constraints (deg/s, deg/s^2)
	public static TrapezoidProfile driveProfile = new TrapezoidProfile(DRIVE_CONSTRAINTS);

	public static Constraints TURN_CONSTRAINTS = new Constraints(20, 80); // FIXME. turn constraints (deg/s, deg/s^2)
	// public static Constraints TURN_CONSTRAINTS = new Constraints(1e9, 1e9);
	public static TrapezoidProfile turnProfile = new TrapezoidProfile(TURN_CONSTRAINTS);

	public SwerveHW hw = new SwerveHW();
	public SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();
	public String name;

	public SwerveModuleState currentState = new SwerveModuleState();
	public double currentAngle;
	public double currentVel;
	public double currentDrivePos;

	public SwerveModuleState targetState = new SwerveModuleState();
	public State setpointAngle = new State();
	public double setpointVel = 0;

	public void init(SwerveModuleConfiguration config) {
		name = config.name;

		hw.init(config);
		sense();
		setpointAngle = new State(inputs.turnAbsPos, inputs.turnVel);
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs(hw.logRoot, inputs);

		currentState = new SwerveModuleState(inputs.driveVel, new Rotation2d(angleModulus(inputs.turnAbsPos)));
		currentAngle = inputs.turnAbsPos;
		currentVel = inputs.driveVel;
		currentDrivePos = inputs.drivePos;
	}

	public void drive(SwerveModuleState state) {
		state.optimize(new Rotation2d(inputs.turnAbsPos));
		targetState = state;
	}

	public void actuate() {
		double targetAngle = targetState.angle.getRadians();
		targetAngle = setpointAngle.position + angleModulus(targetAngle - setpointAngle.position);
		double targetVel = targetState.speedMetersPerSecond * cos(currentAngle - targetAngle);

		State newSetpointAngle = turnProfile.calculate(LOOP_PERIOD, setpointAngle, new State(targetAngle, 0));
		double turnFF = TURN_FF.calculateWithVelocities(setpointAngle.velocity, newSetpointAngle.velocity);
		setpointAngle = newSetpointAngle;

		double newSetpointVel =
				driveProfile.calculate(LOOP_PERIOD, new State(setpointVel, 0), new State(targetVel, 0)).position;
		double driveFF = DRIVE_FF.calculateWithVelocities(setpointVel, newSetpointVel);
		setpointVel = newSetpointVel;

		hw.actuate(inputs, targetVel, driveFF, setpointAngle.position, turnFF);

		Logger.recordOutput(hw.logRoot + "/targetAngle", angleModulus(targetAngle));
		Logger.recordOutput(hw.logRoot + "/targetVel", targetVel);
		Logger.recordOutput(hw.logRoot + "/targetVelCos", targetVel);
		Logger.recordOutput(hw.logRoot + "/turnProfiledPos", angleModulus(setpointAngle.position));
		Logger.recordOutput(hw.logRoot + "/turnProfiledVel", setpointAngle.velocity);
		Logger.recordOutput(hw.logRoot + "/driveFF", driveFF);
		Logger.recordOutput(hw.logRoot + "/turnFF", turnFF);
	}
}
