package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDown extends Command {
	public static double HIGH_VOLTAGE = 10.0; // FIXME. high intake voltage (V)
	public static double CURRENT_THRESHOLD = 30.0; // FIXME. stall/current detection threshold (A)

	public IntakeDown() {
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		double currentPos = intake.inputs.jointPos; // in radians
		intake.setJointPosition(currentPos + Math.toRadians(20.0));
		intake.setIntakeVoltage(HIGH_VOLTAGE);
	}

	@Override
	public boolean isFinished() {
		return intake.inputs.intakeCurrent > CURRENT_THRESHOLD;
	}

	@Override
	public void end(boolean interrupted) {
		intake.setJointPosition(0);
		intake.zeroJointPosition();
	}
}
