package frc.robot.subsystem.drive;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
	public static final double DRIVE_kS = 0.975; // TUNEME. feedforward
	public static final double DRIVE_kV = 2.010;
	public static final double DRIVE_kA = 0.144;
	public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(DRIVE_kS, DRIVE_kV, DRIVE_kA);

	public static final double MAX_VEL = 6.0; // TUNEME. max wheel velocity (m/s)

	public static final Constraints DRIVE_CONSTRAINTS = new Constraints(10, 1e9); // TUNEME
	public static TrapezoidProfile driveProfile = new TrapezoidProfile(DRIVE_CONSTRAINTS);

	public SwerveHW hw = new SwerveHW();
	public SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();
	public String name;

	public SwerveModuleState state = new SwerveModuleState();
	public double angle;
	public double vel;
	public double drivePos;

	public SwerveModuleState targetState = new SwerveModuleState();
	public double setpointVel = 0;

	public void init(SwerveModuleConfiguration config) {
		name = config.name;

		hw.init(config);
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs(hw.logRoot, inputs);

		state = new SwerveModuleState(inputs.driveVelMetersPerSec, new Rotation2d(inputs.turnAbsPosRadians));
		angle = inputs.turnAbsPosRadians;
		vel = inputs.driveVelMetersPerSec;
		drivePos = inputs.drivePosMeters;

		if (robot.isDisabled()) {
			targetState = state;
			setpointVel = 0;
		}
	}

	public void setDriveNeutralMode(NeutralModeValue mode) {
		hw.setDriveNeutralMode(mode);
	}

	public void drive(SwerveModuleState s) {
		targetState = s;
	}

	public void actuate() {
		targetState.optimize(new Rotation2d(inputs.turnAbsPosRadians));
		double targetAng = targetState.angle.getRadians();

		// cos² correction: reduce drive speed when module is misaligned, preserve sign
		double cosErr = cos(angleModulus(angle - targetAng));
		double targetVel = targetState.speedMetersPerSecond * cosErr * abs(cosErr);

		double newSetpointVel =
				driveProfile.calculate(LOOP_PERIOD, new State(setpointVel, 0), new State(targetVel, 0)).position;
		double driveFF = DRIVE_FF.calculateWithVelocities(setpointVel, newSetpointVel);
		setpointVel = newSetpointVel;

		hw.actuate(inputs, setpointVel, driveFF, targetAng);

		Logger.recordOutput(hw.logRoot + "/setpointVelMetersPerSec", setpointVel);
		Logger.recordOutput(hw.logRoot + "/targetVelMetersPerSec", targetVel);
		Logger.recordOutput(hw.logRoot + "/targetAngRadians", angleModulus(targetAng));
		Logger.recordOutput(hw.logRoot + "/driveFFVolts", driveFF);
	}
}
