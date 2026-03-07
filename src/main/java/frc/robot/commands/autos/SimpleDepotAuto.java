package frc.robot.commands.autos;

import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class SimpleDepotAuto extends ParallelCommandGroup {
	// Choreo waypoints (blue alliance)
	public static final double START_X = 3.672;
	public static final double START_Y = 6.862;
	public static final double START_HEADING = PI / 2;

	public static final double DEPOT_X = 0.370;
	public static final double DEPOT_Y = 5.053;

	public static final double END_X = 0.359;
	public static final double END_Y = 6.784;
	public static final double END_HEADING = PI / 2;

	public static final double SPEED = 0.5; // m/s

	public SimpleDepotAuto() {
		// addCommands(
		// 		// Drive sequence: zero shooter, reset pose, drive to depot, turn, drive to end
		// 		Commands.sequence(
		// 				new ShooterStartup(),
		// 				new ResetPose(START_X, START_Y, START_HEADING),
		// 				new DriveForward(START_X, START_Y, DEPOT_X, DEPOT_Y, SPEED, 10),
		// 				new TurnAngle(END_HEADING),
		// 				new DriveForward(DEPOT_X, DEPOT_Y, END_X, END_Y, SPEED, 8)),
		// 		// Run continuously until auto is interrupted
		// 		new IntakeDown(),
		// 		new ShootOnTheFly(),
		// 		new RunIndexer());
	}
}
