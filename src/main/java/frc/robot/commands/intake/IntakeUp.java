package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeUp extends Command {
	public static double LOW_VOLTAGE = 3.0; // FIXME. low intake voltage (V)
	public static double UP_POSITION = Math.toRadians(60.0); // FIXME. joint up position (radians)

	public IntakeUp() {
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		intake.setJointPosition(UP_POSITION);
		intake.setIntakeVoltage(LOW_VOLTAGE);
	}

	@Override
	public void end(boolean interrupted) {
		intake.setIntakeVoltage(0);
	}
}
