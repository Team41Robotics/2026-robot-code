package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Climb extends SequentialCommandGroup {
	public Climb() {
		addCommands(new ClimberUp(), new ClimberDown());
	}
}
