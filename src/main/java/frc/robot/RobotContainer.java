package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drive.FieldHeadingDrive;
import frc.robot.commands.drive.FieldOrientedDrive;
import frc.robot.commands.drive.FieldSnakeDrive;
import frc.robot.commands.drive.RobotOrientedDrive;
import frc.robot.subsystem.controls.Controls;
import frc.robot.subsystem.controls.JoystickControls;
import frc.robot.subsystem.drive.SwerveDrive;
import frc.robot.subsystem.imu.IMU;
import frc.robot.subsystem.vision.Vision;
import frc.robot.test.drive.DrivePIDTestCommand;
import frc.robot.test.drive.TurnPIDTestCommand;

public class RobotContainer {
	public static final double LOOP_PERIOD = 0.020;

	public static Controls controls = new JoystickControls();
	// public static Controls controls = new XboxControls();

	public static Robot robot;
	public static CANBus driveBus = new CANBus("Ducky"); // FIXME.

	public static SwerveDrive drive = new SwerveDrive();
	public static IMU imu = new IMU();
	public static Vision vision = new Vision();

	public static Command autonomousCommand = null;

	public static void init() {
		imu.init();

		drive.init(new Pose2d());
		drive.setDefaultCommand(new FieldOrientedDrive());

		// left_js.button(1).onTrue(new PrintSwervePos());

		// DriveSysID sysid = new DriveSysID();
		// TurnSysID sysid = new TurnSysID();
		// sysid.init();

		SmartDashboard.putData("DrivePIDTest", new DrivePIDTestCommand());
		SmartDashboard.putData("TurnPIDTest", new TurnPIDTestCommand());

		SmartDashboard.putData("RobotOrientedDrive", new RobotOrientedDrive());
		SmartDashboard.putData("FieldOrientedDrive", new FieldOrientedDrive());
		SmartDashboard.putData("FieldHeadingDrive", new FieldHeadingDrive());
		SmartDashboard.putData("FieldSnakeDrive", new FieldSnakeDrive());

		SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
	}

	public static void periodic() {
		imu.sense();
		drive.sense();
		vision.sense();

		CommandScheduler.getInstance().run();

		drive.actuate();
	}

	public static Command getAutonomousCommand() {
		return null;
	}

	public static boolean isRed() {
		return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
	}
}
