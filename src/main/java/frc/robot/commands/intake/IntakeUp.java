package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeUp extends Command {
	public static final double JOINT_UP_VOLTAGE = 7.0; // TUNEME. voltage to hold up (V)

	public IntakeUp() {
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		intake.targetJointVoltage = JOINT_UP_VOLTAGE;
		intake.targetIntakeVoltage = 0;
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetJointVoltage = 0;
		intake.targetIntakeVoltage = 0;
	}
}
