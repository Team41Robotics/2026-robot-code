package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.drive.FieldOrientedDrive;
import frc.robot.commands.drive.RobotOrientedDrive;
import frc.robot.subsystem.drive.SwerveDriveSubsystem;
import frc.robot.test.drive.DrivePIDTestCommand;
import frc.robot.test.drive.TurnPIDTestCommand;

public class RobotContainer {
	public static double LOOP_PERIOD = 0.02;

	public static Robot robot;
	public static SwerveDriveSubsystem drive = new SwerveDriveSubsystem();

	public static CommandJoystick left_js = new CommandJoystick(3);
	public static CommandJoystick right_js = new CommandJoystick(4);
	public static CommandJoystick ds = new CommandJoystick(2);
	public static IMU imu = new IMU();

	public static Command autonomousCommand = null;

	public static void init() {
		drive.init(new Pose2d());
		drive.subsystem.setDefaultCommand(new FieldOrientedDrive());

		// left_js.button(1).onTrue(new PrintSwervePos());

		// DriveSysID sysid = new DriveSysID();
		// TurnSysID sysid = new TurnSysID();
		// sysid.init();

		SmartDashboard.putData("DrivePIDTest", new DrivePIDTestCommand());
		SmartDashboard.putData("TurnPIDTest", new TurnPIDTestCommand());

		SmartDashboard.putData("RobotOrientedDrive", new RobotOrientedDrive());
		SmartDashboard.putData("FieldOrientedDrive", new FieldOrientedDrive());
	}

	public static Command getAutonomousCommand() {
		return null;
	}

	public static boolean isRed() {
		return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
	}
}
