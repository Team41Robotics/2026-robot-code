package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystem.drive.DriveSysID;
import frc.robot.subsystem.drive.SwerveDriveSubsystem;

public class RobotContainer {
	public static Robot robot;

	public static SwerveDriveSubsystem drive = new SwerveDriveSubsystem();

	public static CommandJoystick left_js = new CommandJoystick(3);
	public static CommandJoystick right_js = new CommandJoystick(4);
	public static CommandJoystick ds = new CommandJoystick(2);

	public static IMU imu = new IMU();

	public static void init() {
		drive.init(new Pose2d());

		DriveSysID sysid = new DriveSysID();
		// TurnSysID sysid = new TurnSysID();
		sysid.init();
	}

	public static Command getAutonomousCommand() {
		return null;
	}
}
