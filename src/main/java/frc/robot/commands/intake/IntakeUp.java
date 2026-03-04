package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeUp extends Command {
	public static final double LOW_VOLTAGE = 1.0; // TUNEME. low intake voltage (V)
	public static final double UP_POSITION = 80 / 180.0 * PI;

	public IntakeUp() {
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		intake.targetJointPosition = UP_POSITION;
		intake.targetIntakeVoltage = LOW_VOLTAGE;
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetIntakeVoltage = 0;
	}
}
