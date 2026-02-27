package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDown extends Command {
	public static final double HIGH_VOLTAGE = 10.0; // TUNEME. high intake voltage (V)
	public static final double CURRENT_THRESHOLD = 30.0; // TUNEME. stall/current detection threshold (A)

	public IntakeDown() {
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		double currentPos = intake.inputs.jointPosRadians;
		intake.targetJointPosition = currentPos - 20.0 / 180.0 * PI;
		intake.targetIntakeVoltage = HIGH_VOLTAGE;
	}

	@Override
	public boolean isFinished() {
		return intake.inputs.intakeCurrentAmps > CURRENT_THRESHOLD;
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetJointPosition = 0;
		intake.zeroJointPosition();
	}
}
