package frc.robot.subsystem.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
	public IntakeHW hw = new IntakeHW();
	public IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

	public double targetJointPosition = 0;
	public double targetIntakeVoltage = 0;

	public static final Constraints JOINT_CONSTRAINTS = new Constraints(2.0, 10.0); // FIXME. max vel/accel
	public static TrapezoidProfile jointProfile = new TrapezoidProfile(JOINT_CONSTRAINTS);
	public State jointSetpoint = new State();

	public void init() {
		hw.init();
		sense();

		jointSetpoint = new State(inputs.jointPosRadians, inputs.jointVelRadiansPerSec);
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Intake", inputs);
	}

	public void zeroJointPosition() {
		hw.zeroJointPosition();
	}

	public void actuate() {
		State targetState = new State(targetJointPosition, 0);
		State newSetpoint = jointProfile.calculate(LOOP_PERIOD, jointSetpoint, targetState);
		jointSetpoint = newSetpoint;

		Logger.recordOutput("/Intake/intakeVoltageVolts", targetIntakeVoltage);
		Logger.recordOutput("/Intake/jointTargetPosRadians", targetJointPosition);
		Logger.recordOutput("/Intake/jointProfilePosRadians", jointSetpoint.position);
		Logger.recordOutput("/Intake/jointProfileVelRadiansPerSec", jointSetpoint.velocity);

		hw.actuate(inputs, jointSetpoint.position, targetIntakeVoltage);
	}
}
