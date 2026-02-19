package frc.robot.commands.autos;

import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.climber.PrepareClimb;

public class SimpleClimbAuto extends SequentialCommandGroup {
	public SimpleClimbAuto() {
		addCommands(
                                new ResetPose(3.5, 4.2, 0),
				new PrepareClimb(),
				new TurnAngle(PI / 2),
				new DriveForward(3.5, 4.2, 1.5, 4.2, 0.2, 10.0),
				new Climb());
	}
}
