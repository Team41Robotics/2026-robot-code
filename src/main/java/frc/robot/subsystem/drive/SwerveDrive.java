package frc.robot.subsystem.drive;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
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
	public static final double ROBOT_LEN = 20.750 * 2.54 / 100.;
	public static final double ROBOT_WID = 23.3125 * 2.54 / 100.;

	public static final double MAX_VEL = SwerveModule.MAX_VEL;
	public static final double MAX_W = MAX_VEL / hypot(ROBOT_LEN / 2, ROBOT_WID / 2);

	public SwerveModuleConfiguration[] configs = new SwerveModuleConfiguration[] {
		new SwerveModuleConfiguration("NW", 19, 17, 18, 2.8746799964976915 + PI),
		new SwerveModuleConfiguration("NE", 20, 22, 21, 1.7564080021290591),
		new SwerveModuleConfiguration("SW", 14, 16, 15, 0.8237476830945893 + PI),
		new SwerveModuleConfiguration("SE", 13, 11, 12, -0.6519418348513975)
	};

	public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			new Translation2d(ROBOT_LEN / 2, ROBOT_WID / 2),
			new Translation2d(ROBOT_LEN / 2, -ROBOT_WID / 2),
			new Translation2d(-ROBOT_LEN / 2, ROBOT_WID / 2),
			new Translation2d(-ROBOT_LEN / 2, -ROBOT_WID / 2));

	public SwerveModule[] modules = new SwerveModule[configs.length];

	public SwerveDrivePoseEstimator3d poseEst;

	public SwerveModulePosition[] modulePos = new SwerveModulePosition[configs.length];
	public SwerveModuleState[] targetStates = new SwerveModuleState[configs.length];
	public SwerveModuleState[] measuredStates = new SwerveModuleState[configs.length];
	public ChassisSpeeds targetSpeeds = new ChassisSpeeds();
	public ChassisSpeeds measuredSpeeds = new ChassisSpeeds();

	public Pose3d pose = new Pose3d();
	public Rotation2d rot = new Rotation2d();

	public void init(Pose3d initPose) {
		for (int i = 0; i < modules.length; i++) {
			modules[i] = new SwerveModule();
			modules[i].init(configs[i]);
		}

		SwerveModulePosition[] zeroPos = new SwerveModulePosition[configs.length];
		for (int i = 0; i < configs.length; i++) zeroPos[i] = new SwerveModulePosition();
		poseEst = new SwerveDrivePoseEstimator3d(
				kinematics,
				imu.rotation3d,
				zeroPos,
				initPose,
				VecBuilder.fill(0.1, 0.1, 0.1, 0.05), // TUNEME. odometry state covariance
				VecBuilder.fill(0.75, 0.75, 0.75, 0.9)); // TUNEME. vision measurement covariance (tune)

		sense();
	}

	public void resetPose(Pose3d newPose) {
		poseEst.resetPosition(imu.rotation3d, modulePos, newPose);
		pose = newPose;
		rot = newPose.getRotation().toRotation2d();
	}

	public void drive(ChassisSpeeds speeds) {
		targetSpeeds = speeds;
	}

	public void sense() {
		for (int i = 0; i < modules.length; i++) {
			modules[i].sense();
		}

		for (int i = 0; i < configs.length; i++) {
			modulePos[i] = new SwerveModulePosition(modules[i].drivePos, new Rotation2d(modules[i].angle));
			measuredStates[i] = modules[i].state;
			targetStates[i] = modules[i].targetState;
		}
		measuredSpeeds = kinematics.toChassisSpeeds(measuredStates);

		poseEst.updateWithTime(Timer.getTimestamp(), imu.rotation3d, modulePos);
		pose = poseEst.getEstimatedPosition();
		rot = pose.getRotation().toRotation2d();

		Logger.recordOutput("/Swerve/targetSpeeds", targetSpeeds);
		Logger.recordOutput("/Swerve/measuredSpeeds", measuredSpeeds);
		Logger.recordOutput(
				"/Swerve/realSpeedMetersPerSecond",
				hypot(measuredSpeeds.vxMetersPerSecond, measuredSpeeds.vyMetersPerSecond));
		Logger.recordOutput("/Odom/pose", pose);
		Logger.recordOutput("/Odom/rot", pose.getRotation());
		Logger.recordOutput("/Odom/xMeters", pose.getX());
		Logger.recordOutput("/Odom/yMeters", pose.getY());
		Logger.recordOutput("/Odom/rotRadians", rot.getRadians());
		Logger.recordOutput("/Odom/imuYawRadians", imu.yaw);
		Logger.recordOutput("/Swerve/targetModuleStates", targetStates);
		Logger.recordOutput("/Swerve/measuredModuleStates", measuredStates);

		field.getRobotObject().setPose(pose.toPose2d());
	}

	public void actuate() {
		ChassisSpeeds speeds = ChassisSpeeds.discretize(targetSpeeds, LOOP_PERIOD);
		targetStates = kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveModule.MAX_VEL);

		for (int i = 0; i < modules.length; i++) {
			modules[i].drive(targetStates[i]);
			modules[i].actuate();
		}
	}
}
