package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.shooter.ShooterStartup;

public class StupidShootAuto extends SequentialCommandGroup {
	public StupidShootAuto() {
		addCommands(
				new ShooterStartup(),
				// new InstantCommand(() -> {
				// 	shooter.targetTurretPos = 0.468;
				// 	shooter.targetFlywheelRPM = 2056; // TUNEME
				// 	shooter.targetHoodPos = 10 / 180.0 * PI;
				// }),
				new WaitCommand(5),
				new RunIndexer());
	}
}
