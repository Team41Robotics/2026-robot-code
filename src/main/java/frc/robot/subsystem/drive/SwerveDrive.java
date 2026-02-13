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
	public static final double ROBOT_LEN = 28 * 2.54 / 100.; // FIXME. tunable robot dimensions (m)
	public static final double ROBOT_WID = 27 * 2.54 / 100.; // FIXME. tunable robot dimensions (m)

	public static final double MAX_VEL = SwerveModule.MAX_VEL; // FIXME. max wheel velocity (m/s)
	public static final double MAX_W =
			MAX_VEL / hypot(ROBOT_LEN / 2, ROBOT_WID / 2); // FIXME. derived max angular vel (rad/s)

	public SwerveModuleConfiguration[] configs = new SwerveModuleConfiguration[] {
		new SwerveModuleConfiguration("NW", 9, 10, 15, -0.006135923151542565), // FIXME. ports & angle offset
		new SwerveModuleConfiguration("NE", 7, 8, 18, 0.006135923151542565), // FIXME. ports & angle offset
		new SwerveModuleConfiguration("SW", 11, 12, 17, 0.009203884727313847), // FIXME. ports & angle offset
		new SwerveModuleConfiguration("SE", 5, 6, 16, -0.032213596545598466) // FIXME. ports & angle offset
	};

	public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			new Translation2d(ROBOT_LEN / 2, ROBOT_WID / 2),
			new Translation2d(ROBOT_LEN / 2, -ROBOT_WID / 2),
			new Translation2d(-ROBOT_LEN / 2, ROBOT_WID / 2),
			new Translation2d(-ROBOT_LEN / 2, -ROBOT_WID / 2));

	public SwerveModule[] modules = new SwerveModule[configs.length];

	public SwerveDrivePoseEstimator poseEst;

	public SwerveModulePosition[] modulePos = new SwerveModulePosition[configs.length];
	public SwerveModuleState[] targetStates = new SwerveModuleState[configs.length];
	public SwerveModuleState[] measuredStates = new SwerveModuleState[configs.length];
	public ChassisSpeeds targetSpeeds = new ChassisSpeeds();
	public ChassisSpeeds measuredSpeeds = new ChassisSpeeds();

	public Pose2d pose = new Pose2d();

	public void init(Pose2d initPose) {
		for (int i = 0; i < modules.length; i++) {
			modules[i] = new SwerveModule();
			modules[i].init(configs[i]);
		}

		SwerveModulePosition[] zeroPos = new SwerveModulePosition[configs.length];
		for (int i = 0; i < configs.length; i++) zeroPos[i] = new SwerveModulePosition();
		poseEst = new SwerveDrivePoseEstimator(
				kinematics,
				new Rotation2d(imu.yaw),
				zeroPos,
				initPose,
				VecBuilder.fill(0.1, 0.1, 0.05), // FIXME. odometry state covariance
				VecBuilder.fill(0.75, 0.75, 0.9)); // FIXME. vision measurement covariance (tune)

		sense();
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
			modulePos[i] =
					new SwerveModulePosition(modules[i].drivePos, new Rotation2d(angleModulus(modules[i].angle)));
			measuredStates[i] = modules[i].state;
			targetStates[i] = modules[i].targetState;
		}
		measuredSpeeds = kinematics.toChassisSpeeds(measuredStates);

		poseEst.updateWithTime(Timer.getTimestamp(), new Rotation2d(imu.yaw), modulePos);
		pose = poseEst.getEstimatedPosition();

		Logger.recordOutput("/Swerve/targetSpeeds", targetSpeeds);
		Logger.recordOutput("/Swerve/measuredSpeeds", measuredSpeeds);
		Logger.recordOutput("/Odom/pose", pose);
		Logger.recordOutput("/Odom/rot", pose.getRotation());
		Logger.recordOutput("/Odom/x", pose.getX());
		Logger.recordOutput("/Odom/y", pose.getY());
		Logger.recordOutput("/Odom/rotRad", pose.getRotation().getRadians());
		Logger.recordOutput("/Odom/imuYaw", imu.yaw);
		Logger.recordOutput("/Swerve/targetModuleStates", targetStates);
		Logger.recordOutput("/Swerve/measuredModuleStates", measuredStates);
	}

	public void actuate() {
		for (int i = 0; i < modules.length; i++) modules[i].actuate();
	}
}
