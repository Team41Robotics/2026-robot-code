package frc.robot.subsystem.drive;

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
import frc.robot.Robot;

public class SwerveDriveSubsystem extends SubsystemBase {

	public static double ROBOT_LENGTH = 28 * 2.54;
	public static double ROBOT_WIDTH = 27 * 2.54;

	public static double MAX_VEL = SwerveModule.MAX_VEL;
	public static double MAX_OMEGA = MAX_VEL / hypot(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);

	public SwerveModuleConfiguration[] configs = new SwerveModuleConfiguration[] {
		new SwerveModuleConfiguration("NW", 9, 10, 15, 0),
		new SwerveModuleConfiguration("NE", 7, 8, 18, 0),
		new SwerveModuleConfiguration("SW", 11, 12, 17, 0),
		new SwerveModuleConfiguration("SE", 5, 6, 16, 0)
	};

	public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
			new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),
			new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
			new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2));

	public SwerveModule[] modules;

	public SwerveDrivePoseEstimator pose_est;

	public void init(Pose2d init_pose) {
		modules = new SwerveModule[configs.length];
		for (int i = 0; i < modules.length; i++) {
			modules[i] = new SwerveModule();
			modules[i].init(configs[i]);
		}
		pose_est = new SwerveDrivePoseEstimator(
				kinematics,
				new Rotation2d(imu.yaw()),
				getPositions(),
				init_pose,
				VecBuilder.fill(0.1, 0.1, 0.1),
				VecBuilder.fill(0.75, 0.75, 0.9));
	}

	public SwerveModulePosition[] getPositions() {
		SwerveModulePosition[] pos = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) {
			pos[i] = new SwerveModulePosition(modules[i].currentDrivePos, new Rotation2d(modules[i].currentAngle));
		}
		return pos;
	}

	public ChassisSpeeds targetSpeeds = new ChassisSpeeds(0, 0, 0);

	public void drive(ChassisSpeeds speeds) {
		speeds = ChassisSpeeds.discretize(speeds, Robot.kDefaultPeriod);
		targetSpeeds = speeds;

		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveModule.MAX_VEL);

		for (int i = 0; i < modules.length; i++) {
			modules[i].drive(states[i]);
		}
	}

	@Override
	public void periodic() {
		for (int i = 0; i < modules.length; i++) {
			modules[i].periodic();
		}
		pose_est.updateWithTime(Timer.getTimestamp(), new Rotation2d(imu.yaw()), getPositions());
	}
}
