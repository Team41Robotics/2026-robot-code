package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drive.DrivePIDTestCommand;
import frc.robot.commands.drive.FieldHeadingDrive;
import frc.robot.commands.drive.FieldOrientedDrive;
import frc.robot.commands.drive.FieldSnakeDrive;
import frc.robot.commands.drive.PrintSwervePos;
import frc.robot.commands.drive.RobotOrientedDrive;
import frc.robot.commands.drive.TurnPIDTestCommand;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.intake.IntakeOscillate;
import frc.robot.commands.intake.IntakeUp;
import frc.robot.commands.shooter.ShooterIdle;
import frc.robot.subsystem.controls.Controls;
import frc.robot.subsystem.controls.JoystickControls;
import frc.robot.subsystem.drive.SwerveDrive;
import frc.robot.subsystem.imu.IMU;
import frc.robot.subsystem.indexer.Indexer;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.vision.Vision;

public class RobotContainer {
	public static final double LOOP_PERIOD = 0.020;

	public static Controls controls = new JoystickControls();
	// public static Controls controls = new XboxControls();

	public static Robot robot;
	public static CANBus driveBus = new CANBus("Ducky"); // TUNEME.

	public static SwerveDrive drive = new SwerveDrive();
	public static IMU imu = new IMU();
	public static Vision vision = new Vision();
	public static Intake intake = new Intake();
	public static Shooter shooter = new Shooter();
	public static Indexer indexer = new Indexer();

	public static Command autonomousCommand = null;

	public static void init() {
		imu.init();
		vision.init();
		intake.init();
		shooter.init();
		indexer.init();

		drive.init(new Pose2d());
		// drive.setDefaultCommand(new FieldOrientedDrive());
		drive.setDefaultCommand(new RobotOrientedDrive());
		shooter.setDefaultCommand(new ShooterIdle());

		controls.shoot().onTrue(new PrintSwervePos());

		// DriveSysID sysid = new DriveSysID();
		// TurnSysID sysid = new TurnSysID();
		// sysid.init();

		SmartDashboard.putData("DrivePIDTest", new DrivePIDTestCommand());
		SmartDashboard.putData("TurnPIDTest", new TurnPIDTestCommand());

		SmartDashboard.putData("RobotOrientedDrive", new RobotOrientedDrive());
		SmartDashboard.putData("FieldOrientedDrive", new FieldOrientedDrive());
		SmartDashboard.putData("FieldHeadingDrive", new FieldHeadingDrive());
		SmartDashboard.putData("FieldSnakeDrive", new FieldSnakeDrive());

		SmartDashboard.putData("IntakeDown", new IntakeDown());
		SmartDashboard.putData("IntakeUp", new IntakeUp());
		SmartDashboard.putData("IntakeOscillate", new IntakeOscillate());

		SmartDashboard.putData("RunIndexer", new RunIndexer());

		controls.shoot().whileTrue(new RunIndexer());

		SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
	}

	public static void periodic() {
		imu.sense();
		drive.sense();
		intake.sense();
		shooter.sense();
		indexer.sense();
		vision.sense();

		CommandScheduler.getInstance().run();

		drive.actuate();
		intake.actuate();
		shooter.actuate();
		indexer.actuate();
	}

	public static Command getAutonomousCommand() {
		return null;
	}

	public static boolean isRed() {
		return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
	}
}
