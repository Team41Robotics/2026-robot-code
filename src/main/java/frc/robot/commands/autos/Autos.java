package frc.robot.commands.autos;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.choreo.ChoreoTraj;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.shooter.ShootTeleop;
import frc.robot.commands.shooter.ShooterStartup;
import org.littletonrobotics.junction.Logger;

public class Autos {
	public static AutoFactory factory;

	public static SendableChooser<Pose2d> startPoseChooser = new SendableChooser<>();

	// TUNEME: trajectory tracking PID gains
	public static PIDController xController = new PIDController(5.0, 0, 0);
	public static PIDController yController = new PIDController(5.0, 0, 0);
	public static PIDController thetaController = new PIDController(3.0, 0, 0);

	public static void init() {
		thetaController.enableContinuousInput(-PI, PI);

		factory = new AutoFactory(() -> drive.pose, drive::resetPose, Autos::choreoController, true, drive);

		startPoseChooser.setDefaultOption("null", new Pose2d());
		// startPoseChooser.addOption("testStart", ChoreoVars.Poses.testStart);
		// startPoseChooser.addOption("TestPath start", ChoreoTraj.TestPath.initialPoseBlue());
		startPoseChooser.addOption("TrenchStart", ChoreoTraj.TrenchAuto.initialPoseBlue());
		startPoseChooser.addOption(
				"PITTEST",
				new Pose2d(
						FieldConstants.Hub.innerCenterPoint.toTranslation2d().plus(new Translation2d(-1, 0)),
						Rotation2d.kCCW_Pi_2));
	}

	public static void choreoController(SwerveSample sample) {
		Pose2d pose = drive.pose;

		double xff = sample.vx;
		double yff = sample.vy;
		double wff = sample.omega;

		double xfb = xController.calculate(pose.getX(), sample.x);
		double yfb = yController.calculate(pose.getY(), sample.y);
		double wfb = thetaController.calculate(pose.getRotation().getRadians(), sample.heading);

		ChassisSpeeds speeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(xff + xfb, yff + yfb, wff + wfb, pose.getRotation());

		Logger.recordOutput("/Auto/targetPose", sample.getPose());
		Logger.recordOutput("/Auto/xError", sample.x - pose.getX());
		Logger.recordOutput("/Auto/yError", sample.y - pose.getY());

		drive.drive(speeds);
	}

	public static AutoRoutine testPath() {
		AutoRoutine routine = factory.newRoutine("TestPath");
		AutoTrajectory traj = ChoreoTraj.TestPath.asAutoTraj(routine);

		routine.active()
				.onTrue(Commands.sequence(new ShooterStartup(), traj.cmd()).repeatedly());

		return routine;
	}

	public static Command runIndexerOnCloseSide() {
		return Commands.run(
				() -> {
					indexer.targetSpinVoltage = RunIndexer.DEFAULT_SPIN_VOLTAGE;
					indexer.targetElevatorVoltage = RunIndexer.DEFAULT_ELEVATOR_VOLTAGE;
				},
				indexer);
	}

	public static AutoRoutine trenchAuto() {
		AutoRoutine routine = factory.newRoutine("TrenchAuto");

		routine.active().onTrue(Commands.sequence(new ShooterStartup(), new ShootTeleop()));
		routine.active().onTrue(Commands.sequence(new WaitCommand(5), runIndexerOnCloseSide()));

		return routine;
	}
}
