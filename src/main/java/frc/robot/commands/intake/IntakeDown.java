package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDown extends Command {
	public static final double HIGH_VOLTAGE = 4.0; // TUNEME. high intake voltage (V)
	public static final double DOWN_POSITION = 12 * PI / 180.0;

	public IntakeDown() {
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		intake.targetJointPosition = DOWN_POSITION;
		intake.targetIntakeVoltage = HIGH_VOLTAGE;
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetIntakeVoltage = 0;
	}
}
