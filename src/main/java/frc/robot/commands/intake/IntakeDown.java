package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDown extends Command {
	public static final double JOINT_kP = 14.3 / 2;
	public static final double JOINT_kD = 0.8;
	public static final double HIGH_VOLTAGE = 12.0;
	public static final double CURRENT_THRESHOLD = 100.0;
	public static final double REVERSE_DURATION_SEC = 0.2;

	public double voltage;
	public boolean reversing = false;
	public double reverseStartSec = 0;

	public IntakeDown() {
		this(HIGH_VOLTAGE);
	}

	public IntakeDown(double voltage) {
		this.voltage = voltage;
		addRequirements(intake);
	}

	@Override
	public void execute() {
		intake.targetJointVoltage =
				-JOINT_kP * intake.inputs.jointPosRadians - JOINT_kD * intake.inputs.jointVelRadiansPerSec;

		double now = Timer.getTimestamp();
		if (reversing) {
			intake.targetIntakeVoltage = -HIGH_VOLTAGE;
			if (now - reverseStartSec >= REVERSE_DURATION_SEC) {
				reversing = false;
			}
		} else {
			intake.targetIntakeVoltage = voltage;
			if (intake.inputs.intakeCurrentAmps >= CURRENT_THRESHOLD) {
				reversing = true;
				reverseStartSec = now;
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetJointVoltage = 0;
		intake.targetIntakeVoltage = 0;
		reversing = false;
	}
}
