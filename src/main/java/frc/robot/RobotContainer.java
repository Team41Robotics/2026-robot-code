package frc.robot;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.autos.Autos;
import frc.robot.commands.autos.StupidShootAuto;
import frc.robot.commands.drive.DrivePIDTestCommand;
import frc.robot.commands.drive.FieldHeadingDrive;
import frc.robot.commands.drive.FieldOrientedDrive;
import frc.robot.commands.drive.FieldSnakeDrive;
import frc.robot.commands.drive.PrintSwervePos;
import frc.robot.commands.drive.RobotOrientedDrive;
import frc.robot.commands.drive.TurnPIDTestCommand;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.intake.IntakeUp;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.controls.Controls;
import frc.robot.subsystem.controls.XboxControls;
import frc.robot.subsystem.drive.SwerveDrive;
import frc.robot.subsystem.imu.IMU;
import frc.robot.subsystem.indexer.Indexer;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.leds.LEDS;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.vision.Vision;

@SuppressWarnings("static-access")
public class RobotContainer {
	public static final double LOOP_PERIOD = 0.020;

	// public static Controls controls = new JoystickControls();
	public static Controls controls = new XboxControls();

	public static Robot robot;
	public static CANBus driveBus = new CANBus("Ducky");

	public static SwerveDrive drive = new SwerveDrive();
	public static IMU imu = new IMU();
	public static Vision vision = new Vision();
	public static Intake intake = new Intake();
	public static Shooter shooter = new Shooter();
	public static Indexer indexer = new Indexer();
	public static LEDS leds = new LEDS();
	public static Climber climber = new Climber();

	public static Field2d field = new Field2d();

	public static AutoChooser autoChooser = new AutoChooser();
	public static Command autonomousCommand = null;

	public static void init() {
		imu.init();
		intake.init();
		shooter.init();
		indexer.init();
		leds.init();
		climber.init();
		drive.init(new Pose2d());
		vision.init();

		drive.setDefaultCommand(new FieldOrientedDrive());
		// drive.setDefaultCommand(new RobotOrientedDrive());
		// shooter.setDefaultCommand(new ShooterIdle());
		intake.setDefaultCommand(new IntakeUp());

		controls.shoot().onTrue(new PrintSwervePos());

		// DriveSysID sysid = new DriveSysID();
		// TurnSysID sysid = new TurnSysID();
		// ShooterFlywheelSysID sysid = new ShooterFlywheelSysID();
		// sysid.init();

		SmartDashboard.putData("DrivePIDTest", new DrivePIDTestCommand());
		SmartDashboard.putData("TurnPIDTest", new TurnPIDTestCommand());

		SmartDashboard.putData("RobotOrientedDrive", new RobotOrientedDrive());
		SmartDashboard.putData("FieldOrientedDrive", new FieldOrientedDrive());
		SmartDashboard.putData("FieldHeadingDrive", new FieldHeadingDrive());
		SmartDashboard.putData("FieldSnakeDrive", new FieldSnakeDrive());

		SmartDashboard.putData("IntakeDown", new IntakeDown());
		SmartDashboard.putData("IntakeUp", new IntakeUp());

		SmartDashboard.putData("RunIndexer", new RunIndexer());

		SmartDashboard.putData("StupidShootAuto", new StupidShootAuto());

		// controls.shoot().whileTrue(new RunIndexer());
		controls.intake().whileTrue(new IntakeDown());
		controls.shoot().whileTrue(new RunIndexer());

		Autos.init();
		autoChooser.addRoutine("TestPath", Autos::testPath);
		SmartDashboard.putData("AutoChooser", autoChooser);

		SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
		SmartDashboard.putData("Field", field);
	}

	public static void periodic() {
		imu.sense();
		drive.sense();
		intake.sense();
		shooter.sense();
		indexer.sense();
		vision.sense();
		climber.sense();
		// leds.sense();

		CommandScheduler.getInstance().run();

		if (DriverStation.isDisabled()) {
			leds.control = leds.DISABLED_ANIMATION;
			autonomousCommand = autoChooser.selectedCommand();
		}

		drive.actuate();
		intake.actuate();
		shooter.actuate();
		indexer.actuate();
		climber.actuate();
		// leds.actuate();
	}

	public static boolean isRed() {
		return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
	}
}
