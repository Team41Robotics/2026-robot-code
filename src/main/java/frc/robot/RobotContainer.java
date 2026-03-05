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
import frc.robot.commands.climber.Climb;
import frc.robot.commands.climber.ClimberDown;
import frc.robot.commands.climber.ClimberUp;
import frc.robot.commands.climber.PrepareClimb;
import frc.robot.commands.drive.DrivePIDTestCommand;
import frc.robot.commands.drive.FieldHeadingDrive;
import frc.robot.commands.drive.FieldOrientedDrive;
import frc.robot.commands.drive.FieldSnakeDrive;
import frc.robot.commands.drive.NoDrive;
import frc.robot.commands.drive.PrintSwervePos;
import frc.robot.commands.drive.RobotOrientedDrive;
import frc.robot.commands.drive.TurnPIDTestCommand;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.indexer.StopIndexer;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.intake.IntakeUp;
import frc.robot.commands.shooter.ManualShoot;
import frc.robot.commands.shooter.ShootOnTheFly;
import frc.robot.commands.shooter.ShooterIdle;
import frc.robot.commands.shooter.ShooterStartup;
import frc.robot.commands.shooter.StaticShootAtTarget;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.controls.Controls;
import frc.robot.subsystem.controls.JoystickControls;
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

	public static Controls controls = new JoystickControls();
	// public static Controls controls = new XboxControls();

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

	public static String currentPeriod = "DISABLED";
	public static double periodTimeRemaining = 0;
	public static String allianceHubStatus = "Unknown";

	static Pose2d prevStartPose = null;

	public static void init() {
		imu.init();
		intake.init();
		shooter.init();
		indexer.init();
		leds.init();
		// climber.init();
		drive.init(new Pose2d());
		vision.init();

		drive.setDefaultCommand(new FieldOrientedDrive());
		// drive.setDefaultCommand(new RobotOrientedDrive());
		shooter.setDefaultCommand(new ShootOnTheFly());
		intake.setDefaultCommand(new IntakeDown());
		indexer.setDefaultCommand(new RunIndexer(0.5, 0));

		// controls.shoot().onTrue(new PrintSwervePos());

		// DriveSysID sysid = new DriveSysID();
		// TurnSysID sysid = new TurnSysID();
		// ShooterFlywheelSysID sysid = new ShooterFlywheelSysID();
		// sysid.init();

		SmartDashboard.putData("DrivePIDTest", new DrivePIDTestCommand());
		SmartDashboard.putData("TurnPIDTest", new TurnPIDTestCommand());

		SmartDashboard.putData("NoDrive", new NoDrive());
		SmartDashboard.putData("RobotOrientedDrive", new RobotOrientedDrive());
		SmartDashboard.putData("FieldOrientedDrive", new FieldOrientedDrive());
		SmartDashboard.putData("FieldHeadingDrive", new FieldHeadingDrive());
		SmartDashboard.putData("FieldSnakeDrive", new FieldSnakeDrive());

		SmartDashboard.putData("IntakeDown", new IntakeDown());
		SmartDashboard.putData("IntakeUp", new IntakeUp());

		SmartDashboard.putData("RunIndexer", new RunIndexer());
		SmartDashboard.putData("StopIndexer", new StopIndexer());

		SmartDashboard.putData("ShooterIdle", new ShooterIdle());
		SmartDashboard.putData("ShooterStartup", new ShooterStartup());
		SmartDashboard.putData("StaticShootAtTarget", new StaticShootAtTarget());
		SmartDashboard.putData("ShootOnTheFly", new ShootOnTheFly());
		SmartDashboard.putData("ManualShoot", new ManualShoot());

		SmartDashboard.putData("PrintSwervePos", new PrintSwervePos());

		SmartDashboard.putData("Climb", new Climb());
		SmartDashboard.putData("ClimberUp", new ClimberUp());
		SmartDashboard.putData("ClimberDown", new ClimberDown());
		SmartDashboard.putData("PrepareClimb", new PrepareClimb());

		SmartDashboard.putData("StupidShootAuto", new StupidShootAuto());

		controls.intake().whileTrue(new IntakeUp());
		controls.shoot().whileTrue(new RunIndexer());

		Autos.init();
		autoChooser.addRoutine("TestPath", Autos::testPath);
		autoChooser.addRoutine("StupidShootAuto", Autos::stupidShootAuto);
		autoChooser.addRoutine("SimpleDepotAuto", Autos::simpleDepotAuto);

		SmartDashboard.putData("AutoChooser", autoChooser);
		SmartDashboard.putData("StartPoseChooser", Autos.startPoseChooser);

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
		// climber.sense();
		// leds.sense();

		CommandScheduler.getInstance().run();

		if (DriverStation.isDisabled()) {
			leds.control = leds.DISABLED_ANIMATION;
			autonomousCommand = autoChooser.selectedCommand();

			Pose2d selected = Autos.startPoseChooser.getSelected();
			if (selected != null && selected != prevStartPose) {
				prevStartPose = selected;
				drive.resetPose(selected);
			}
		}

		updateMatchPeriod();

		drive.actuate();
		intake.actuate();
		shooter.actuate();
		indexer.actuate();
		// climber.actuate();
		// leds.actuate();
	}

	public static boolean redWonAuto() {
		String gameData = DriverStation.getGameSpecificMessage();
		if (gameData != null && gameData.length() > 0) {
			switch (gameData.charAt(0)) {
				case 'R':
					return true;
				case 'B':
					return false;
				default:
					break;
			}
		}
		// No data yet (early teleop or no FMS), assume false
		return false;
	}

	public static boolean isHubActive() {
		if (DriverStation.isAutonomousEnabled()) {
			return true;
		}
		if (!DriverStation.isTeleopEnabled()) {
			return false;
		}
		double matchTime = DriverStation.getMatchTime();
		boolean weWonAuto = (isRed() && redWonAuto()) || (!isRed() && !redWonAuto());

		if (matchTime > 130) {
			return true;
		} else if (matchTime > 105) {
			return !weWonAuto; // Shift 1: auto winner inactive
		} else if (matchTime > 80) {
			return weWonAuto; // Shift 2: auto winner active
		} else if (matchTime > 55) {
			return !weWonAuto; // Shift 3: auto winner inactive
		} else if (matchTime > 30) {
			return weWonAuto; // Shift 4: auto winner active
		} else {
			return true; // End game: always active
		}
	}

	public static void updateMatchPeriod() {
		double matchTime = DriverStation.getMatchTime();

		if (DriverStation.isDisabled()) {
			currentPeriod = "DISABLED";
			periodTimeRemaining = 0;
			allianceHubStatus = "---";
		} else if (DriverStation.isAutonomous()) {
			currentPeriod = "AUTO";
			periodTimeRemaining = matchTime;
			allianceHubStatus = "Active";
		} else if (DriverStation.isTeleop()) {
			boolean weWonAuto = (isRed() && redWonAuto()) || (!isRed() && !redWonAuto());

			if (matchTime > 130) {
				currentPeriod = "TRANSITION";
				periodTimeRemaining = matchTime - 130;
				allianceHubStatus = "Active";
			} else if (matchTime > 105) {
				currentPeriod = "SHIFT 1";
				periodTimeRemaining = matchTime - 105;
				// Odd shift: auto winner's hub inactive
				allianceHubStatus = weWonAuto ? "Inactive" : "Active";
			} else if (matchTime > 80) {
				currentPeriod = "SHIFT 2";
				periodTimeRemaining = matchTime - 80;
				// Even shift: auto winner's hub active
				allianceHubStatus = weWonAuto ? "Active" : "Inactive";
			} else if (matchTime > 55) {
				currentPeriod = "SHIFT 3";
				periodTimeRemaining = matchTime - 55;
				allianceHubStatus = weWonAuto ? "Inactive" : "Active";
			} else if (matchTime > 30) {
				currentPeriod = "SHIFT 4";
				periodTimeRemaining = matchTime - 30;
				allianceHubStatus = weWonAuto ? "Active" : "Inactive";
			} else {
				currentPeriod = "END GAME";
				periodTimeRemaining = matchTime;
				allianceHubStatus = "Active";
			}
		} else {
			currentPeriod = "TEST";
			periodTimeRemaining = 0;
			allianceHubStatus = "---";
		}

		SmartDashboard.putString("MatchPeriod", currentPeriod);
		SmartDashboard.putNumber("PeriodTimeRemaining", periodTimeRemaining);
		SmartDashboard.putString("AllianceHubStatus", allianceHubStatus);
		SmartDashboard.putBoolean("RedWonAuto", redWonAuto());
		SmartDashboard.putString("GameData", DriverStation.getGameSpecificMessage());
	}

	public static boolean isRed() {
		return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
	}
}
