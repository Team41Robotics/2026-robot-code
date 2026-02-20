package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class StartIntake extends Command {
	public double voltage;

	public StartIntake() {
		this(10.0); // FIXME. default intake voltage (V)
	}

	public StartIntake(double voltage) {
		this.voltage = voltage;
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		intake.setVoltage(voltage);
	}

	@Override
	public void execute() {}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		intake.stop();
	}
}
