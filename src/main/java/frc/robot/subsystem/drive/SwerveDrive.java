package frc.robot.subsystem.drive;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {
	public static double ROBOT_LENGTH = 28 * 2.54 / 100.;
	public static double ROBOT_WIDTH = 27 * 2.54 / 100.;

	public static double MAX_VEL = SwerveModule.MAX_VEL;
	public static double MAX_OMEGA = MAX_VEL / hypot(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);

	public SwerveModuleConfiguration[] configs = new SwerveModuleConfiguration[] {
		new SwerveModuleConfiguration("NW", 9, 10, 15, -0.006135923151542565),
		new SwerveModuleConfiguration("NE", 7, 8, 18, 0.006135923151542565),
		new SwerveModuleConfiguration("SW", 11, 12, 17, 0.009203884727313847),
		new SwerveModuleConfiguration("SE", 5, 6, 16, -0.032213596545598466)
	};

	public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
			new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),
			new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
			new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2));

	public SwerveModule[] modules;

	public SwerveDrivePoseEstimator pose_est;

	public SwerveModulePosition[] swervePositions = new SwerveModulePosition[configs.length];
	public SwerveModuleState[] targetStates = new SwerveModuleState[configs.length];
	public SwerveModuleState[] swerveStates = new SwerveModuleState[configs.length];
	public ChassisSpeeds targetSpeeds = new ChassisSpeeds();
	public ChassisSpeeds speeds = new ChassisSpeeds();

	public Pose2d pose = new Pose2d();

	public void init(Pose2d init_pose) {
		modules = new SwerveModule[configs.length];
		for (int i = 0; i < modules.length; i++) {
			modules[i] = new SwerveModule();
			modules[i].init(configs[i]);
		}
		SwerveModulePosition[] zeropos = new SwerveModulePosition[configs.length];
		for (int i = 0; i < configs.length; i++) zeropos[i] = new SwerveModulePosition();
		pose_est = new SwerveDrivePoseEstimator(
				kinematics,
				new Rotation2d(imu.yaw),
				zeropos,
				init_pose,
				VecBuilder.fill(0.1, 0.1, 0.05), // TODO
				VecBuilder.fill(0.75, 0.75, 0.9));
	}

	public void drive(ChassisSpeeds speeds) {
		speeds = ChassisSpeeds.discretize(speeds, LOOP_PERIOD);
		targetSpeeds = speeds;

		targetStates = kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveModule.MAX_VEL);

		for (int i = 0; i < modules.length; i++) {
			modules[i].drive(targetStates[i]);
		}
	}

	public void sense() {
		for (int i = 0; i < modules.length; i++) {
			modules[i].sense();
		}

		for (int i = 0; i < configs.length; i++) {
			swervePositions[i] = new SwerveModulePosition(
					modules[i].currentDrivePos, new Rotation2d(angleModulus(modules[i].currentAngle)));
			swerveStates[i] = modules[i].currentState;
			targetStates[i] = modules[i].targetState;
		}
		speeds = kinematics.toChassisSpeeds(swerveStates);

		pose_est.updateWithTime(Timer.getTimestamp(), new Rotation2d(imu.yaw), swervePositions);
		pose = pose_est.getEstimatedPosition();

		Logger.recordOutput("/Swerve/speeds_setpoints", targetSpeeds);
		Logger.recordOutput("/Swerve/speeds", speeds);
		Logger.recordOutput("/Odom/pose", pose);
		Logger.recordOutput("/Odom/rot", pose.getRotation());
		Logger.recordOutput("/Odom/x", pose.getX());
		Logger.recordOutput("/Odom/y", pose.getY());
		Logger.recordOutput("/Odom/rot_raw", pose.getRotation().getRadians());
		Logger.recordOutput("/Odom/IMU_yaw", imu.yaw);
		Logger.recordOutput("/Swerve/module_setpoint", targetStates);
		Logger.recordOutput("/Swerve/module_speeds", swerveStates);
	}

	public void actuate() {
		for (int i = 0; i < modules.length; i++) modules[i].actuate();
	}
}
