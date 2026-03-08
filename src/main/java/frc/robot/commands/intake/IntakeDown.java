package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDown extends Command {
	public static final double JOINT_kP = 14.3/2; // TUNEME. ~3V at 12 degrees
	public static final double JOINT_kD = 0.8; // TUNEME. damping for disturbance rejection
	public static final double HIGH_VOLTAGE = 4.0; // TUNEME. intake roller voltage (V)

	public double voltage;

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
		intake.targetIntakeVoltage = voltage;
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetJointVoltage = 0;
		intake.targetIntakeVoltage = 0;
	}
}
