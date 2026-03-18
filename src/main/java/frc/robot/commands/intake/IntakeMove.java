package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeMove extends Command {
	public static final double INTAKE_VOLTAGE = 12.0; // TUNEME. intake roller voltage (V)
	public static final double EXTENSION_POSITION = 12.0; // TUNEME. extension position (m)

	public double voltage;
	public double extensionPosition;
	
	public IntakeMove() {
		this(INTAKE_VOLTAGE, EXTENSION_POSITION);
	}

	public IntakeMove(double voltage, double extensionPosition) {
		this.voltage = voltage;
		this.extensionPosition = extensionPosition;
		addRequirements(intake);
	}

	@Override
	public void execute() {
		intake.targetExtensionPosition = extensionPosition;
		intake.targetIntakeVoltage = voltage;
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetExtensionPosition = intake.inputs.extensionPosMeters;
		intake.targetIntakeVoltage = 0;
	}
}
