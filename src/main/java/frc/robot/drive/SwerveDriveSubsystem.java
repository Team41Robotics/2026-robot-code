package frc.robot.drive;

import static java.lang.Math.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class SwerveDriveSubsystem extends SubsystemBase {

	public static double ROBOT_LENGTH = 0;
	public static double ROBOT_WIDTH = 0;

	public static SwerveModuleConfiguration[] configs = new SwerveModuleConfiguration[] {
		new SwerveModuleConfiguration("NW", 9, 10, 15, false, 0),
		new SwerveModuleConfiguration("NE", 7, 8, 18, false, 0),
		new SwerveModuleConfiguration("SW", 11, 12, 17, false, 0),
		new SwerveModuleConfiguration("SE", 5, 6, 16, false, 0)
	};

	public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
			new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2),
			new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2),
			new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2));

	public static SwerveModule[] modules;

	public void init() {
		modules = new SwerveModule[modules.length];
		for (int i = 0; i < modules.length; i++) {
			modules[i] = new SwerveModule();
			modules[i].init(configs[i]);
		}
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
	}
}
