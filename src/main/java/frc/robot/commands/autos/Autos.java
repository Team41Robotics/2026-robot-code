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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.Util;
import frc.robot.choreo.ChoreoTraj;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.shooter.ShootTeleop;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
	public static AutoFactory factory;

	public static LoggedDashboardChooser<Pose2d> startPoseChooser = new LoggedDashboardChooser<>("StartPoseChooser");

	// TUNEME: trajectory tracking PID gains
	public static PIDController xController = new PIDController(5.0, 0, 0);
	public static PIDController yController = new PIDController(5.0, 0, 0);
	public static PIDController thetaController = new PIDController(3.0, 0, 0);

	public static void init() {
		thetaController.enableContinuousInput(-PI, PI);

		factory = new AutoFactory(() -> drive.pose, drive::resetPose, Autos::choreoController, true, drive);

		startPoseChooser.addDefaultOption("null", new Pose2d());
		// startPoseChooser.addOption("testStart", ChoreoVars.Poses.testStart);
		// startPoseChooser.addOption("TestPath start", ChoreoTraj.TestPath.initialPoseBlue());
		startPoseChooser.addOption("TrenchStart", ChoreoTraj.TrenchAuto.initialPoseBlue());
		startPoseChooser.addOption(
				"PITTEST",
				new Pose2d(
						FieldConstants.Hub.innerCenterPoint.toTranslation2d().plus(new Translation2d(-1, 0)),
						Rotation2d.kCCW_Pi_2));
		startPoseChooser.onChange(pose -> {
			if (pose != null && pose.getX() != 0 && pose.getY() != 0) {
				drive.resetPose(Util.flipIfRed(pose));
			}
		});
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
		Logger.recordOutput("/Auto/targetVx", sample.vx);
		Logger.recordOutput("/Auto/targetVy", sample.vy);
		Logger.recordOutput("/Auto/targetOmega", sample.omega);
		Logger.recordOutput("/Auto/xError", sample.x - pose.getX());
		Logger.recordOutput("/Auto/yError", sample.y - pose.getY());

		drive.drive(speeds);
	}

	public static boolean isOnCloseSide() {
		Translation2d robotPos = Util.flipIfRed(drive.pose.getTranslation());
		double xTrenchMin = FieldConstants.LinesVertical.hubCenter - FieldConstants.LeftBump.width / 2.0;
		return robotPos.getX() < xTrenchMin;
	}

	public static Command runIndexerOnCloseSide() {
		return Commands.run(
				() -> {
					if (isOnCloseSide() && shooter.onTarget) {
						indexer.targetSpinVoltage = RunIndexer.DEFAULT_SPIN_VOLTAGE;
						indexer.targetElevatorVoltage = RunIndexer.DEFAULT_ELEVATOR_VOLTAGE;
						indexer.targetBackvatorVoltage = RunIndexer.DEFAULT_BACKVATOR_VOLTAGE;
					} else {
						indexer.targetSpinVoltage = 0;
						indexer.targetElevatorVoltage = 0;
						indexer.targetBackvatorVoltage = 0;
					}
				},
				indexer);
	}

	public static AutoRoutine buildAuto(ChoreoTraj choreoTraj) {
		AutoRoutine routine = factory.newRoutine(choreoTraj.name());
		AutoTrajectory traj = choreoTraj.asAutoTraj(routine);

		routine.active()
				.onTrue(Commands.runOnce(() -> Logger.recordOutput("/Auto/selectedTrajectory", choreoTraj.name())));
		routine.active().onTrue(new ShootTeleop());
		routine.active().onTrue(new IntakeDown());
		routine.active().onTrue(traj.cmd().andThen(new WaitCommand(10)));
		routine.active().onTrue(runIndexerOnCloseSide());

		return routine;
	}
}
