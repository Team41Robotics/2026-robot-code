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
	public static final double DRIVE_kS = 0.093052; // TUNEME. feedforward kS (tune)
	public static final double DRIVE_kV = 1.8968; // TUNEME. feedforward kV (tune)
	public static final double DRIVE_kA = 0.15096; // TUNEME. feedforward kA (tune)
	public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(DRIVE_kS, DRIVE_kV, DRIVE_kA);

	public static final double TURN_kS = 0.19431; // TUNEME. turn feedforward kS (tune)
	public static final double TURN_kV = 0.36606; // TUNEME. turn feedforward kV (tune)
	public static final double TURN_kA = 0.; // TUNEME. turn feedforward kA (tune)
	public static final SimpleMotorFeedforward TURN_FF = new SimpleMotorFeedforward(TURN_kS, TURN_kV, TURN_kA);

	public static final double MAX_VEL = 6.0; // TUNEME. max wheel velocity (m/s)

	public static final Constraints DRIVE_CONSTRAINTS =
			new Constraints(45, 1e9); // TUNEME. drive constraints (deg/s, deg/s^2)
	public static TrapezoidProfile driveProfile = new TrapezoidProfile(DRIVE_CONSTRAINTS);

	public static final Constraints TURN_CONSTRAINTS =
			new Constraints(20, 80); // TUNEME. turn constraints (deg/s, deg/s^2)
	public static TrapezoidProfile turnProfile = new TrapezoidProfile(TURN_CONSTRAINTS);

	public SwerveHW hw = new SwerveHW();
	public SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();
	public String name;

	public SwerveModuleState state = new SwerveModuleState();
	public double angle;
	public double vel;
	public double drivePos;

	public SwerveModuleState targetState = new SwerveModuleState();
	public State setpointAng = new State();
	public double setpointVel = 0;

	public void init(SwerveModuleConfiguration config) {
		name = config.name;

		hw.init(config);
		sense();
		setpointAng = new State(inputs.turnAbsPosRadians, inputs.turnVelRadiansPerSec);
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs(hw.logRoot, inputs);

		state = new SwerveModuleState(
				inputs.driveVelMetersPerSec, new Rotation2d(inputs.turnAbsPosRadians));
		angle = inputs.turnAbsPosRadians;
		vel = inputs.driveVelMetersPerSec;
		drivePos = inputs.drivePosMeters;
	}

	public void drive(SwerveModuleState s) {
		s.optimize(new Rotation2d(inputs.turnAbsPosRadians));
		targetState = s;
	}

	public void actuate() {
		double targetAng = targetState.angle.getRadians();
		targetAng = setpointAng.position + angleModulus(targetAng - setpointAng.position);
		double targetVel = targetState.speedMetersPerSecond * cos(angle - targetAng);

		State newSetpointAng = turnProfile.calculate(LOOP_PERIOD, setpointAng, new State(targetAng, 0));
		double turnFF = TURN_FF.calculateWithVelocities(setpointAng.velocity, newSetpointAng.velocity);
		setpointAng = newSetpointAng;

		double newSetpointVel =
				driveProfile.calculate(LOOP_PERIOD, new State(setpointVel, 0), new State(targetVel, 0)).position;
		double driveFF = DRIVE_FF.calculateWithVelocities(setpointVel, newSetpointVel);
		setpointVel = newSetpointVel;

		hw.actuate(inputs, targetVel, driveFF, setpointAng.position, turnFF);

		Logger.recordOutput(hw.logRoot + "/setpointVelMetersPerSec", setpointVel);
		Logger.recordOutput(hw.logRoot + "/targetVelMetersPerSec", targetVel);
		Logger.recordOutput(hw.logRoot + "/targetAngRadians", angleModulus(targetAng));
		Logger.recordOutput(hw.logRoot + "/setpointAngRadians", angleModulus(setpointAng.position));
		Logger.recordOutput(hw.logRoot + "/setpointAngVelRadiansPerSec", setpointAng.velocity);
		Logger.recordOutput(hw.logRoot + "/driveFFVolts", driveFF);
		Logger.recordOutput(hw.logRoot + "/turnFFVolts", turnFF);
	}
}
