package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.PreMatchCheck;
import frc.robot.commands.autos.Autos;
import frc.robot.commands.autos.StupidShootAuto;
import frc.robot.commands.drive.DriveLock;
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
import frc.robot.commands.shooter.HoodZero;
import frc.robot.commands.shooter.ManualShoot;
import frc.robot.commands.shooter.ShootOnTheFly;
import frc.robot.commands.shooter.ShootTeleop;
import frc.robot.commands.shooter.ShooterIdle;
import frc.robot.commands.shooter.ShooterStartup;
import frc.robot.commands.shooter.StaticShootAtTarget;
import frc.robot.subsystem.controls.Controls;
import frc.robot.subsystem.controls.JoystickControls;
import frc.robot.subsystem.drive.SwerveDrive;
import frc.robot.subsystem.imu.IMU;
import frc.robot.subsystem.indexer.Indexer;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.leds.LEDS;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.vision.Vision;
import org.littletonrobotics.junction.Logger;

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

	public static Field2d field = new Field2d();

	public static LoggedAutoChooser autoChooser = new LoggedAutoChooser();
	public static Command autonomousCommand = null;

	public static String currentPeriod = "DISABLED";
	public static double periodTimeRemaining = 0;
	public static String allianceHubStatus = "Unknown";

	static Pose2d prevStartPose = null;

	public static void init() { // TODO change Akit mode to live mode
		imu.init();
		intake.init();
		shooter.init();
		indexer.init();
		leds.init();
		drive.init(new Pose2d());
		vision.init();

		drive.setDefaultCommand(new FieldOrientedDrive());
		shooter.setDefaultCommand(new ShootTeleop());
		intake.setDefaultCommand(new IntakeDown());
		indexer.setDefaultCommand(new RunIndexer(0.5, 0, 0));

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

		SmartDashboard.putData("DriveLock", new DriveLock());
		SmartDashboard.putData("PrintSwervePos", new PrintSwervePos());

		SmartDashboard.putData("StupidShootAuto", new StupidShootAuto());

		SmartDashboard.putData("PreMatchCheck", new PreMatchCheck());
		CommandScheduler.getInstance().schedule(new PreMatchCheck());

		// controls.intake().whileTrue(new IntakeUp());
		controls.shoot().whileTrue(new RunIndexer());

		controls.intakeDown().onTrue(new IntakeDown());
		controls.intakeUp().onTrue(new IntakeUp());
		// controls.intakeReverse().whileTrue(new IntakeDown(-IntakeDown.HIGH_VOLTAGE));
		controls.intakeReverse().whileTrue(new IntakeUp());
		controls.indexerReverse().whileTrue(new RunIndexer(-RunIndexer.DEFAULT_SPIN_VOLTAGE, 0, 0));

		controls.hoodZero().whileTrue(new HoodZero());
		controls.driveLock().whileTrue(new DriveLock());

		// Emergency stops
		controls.eStopShooter()
				.whileTrue(new RunCommand(
						() -> {
							shooter.targetFlywheelRPM = 0;
							shooter.targetTurretPos = shooter.inputs.turretPosRadians;
							shooter.targetHoodPos = shooter.inputs.hoodPosRadians;
							indexer.targetSpinVoltage = 0;
							indexer.targetElevatorVoltage = 0;
						},
						shooter,
						indexer));
		controls.eStopAll()
				.whileTrue(new RunCommand(
						() -> {
							shooter.targetFlywheelRPM = 0;
							shooter.targetTurretPos = shooter.inputs.turretPosRadians;
							shooter.targetHoodPos = shooter.inputs.hoodPosRadians;
							intake.targetJointVoltage = 0;
							intake.targetIntakeVoltage = 0;
							indexer.targetSpinVoltage = 0;
							indexer.targetElevatorVoltage = 0;
						},
						shooter,
						intake,
						indexer));

		Autos.init();
		autoChooser.addRoutine("TestPath", Autos::testPath);
		autoChooser.addRoutine("StupidShootAuto", Autos::stupidShootAuto);
		// autoChooser.addRoutine("SimpleDepotAuto", Autos::simpleDepotAuto);
		autoChooser.addRoutine("DepotAuto", Autos::depotAuto);
		autoChooser.addRoutine("OutpostAuto_1", Autos::outpostAuto1);
		autoChooser.addRoutine("OutpostAuto_2", Autos::outpostAuto2);
		autoChooser.addRoutine("TrenchAuto", Autos::trenchAuto);
		autoChooser.addRoutine("MiddletoHP", Autos::middleToHP);

		SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
		SmartDashboard.putData("Field", field);
	}

	public static void periodic() {
		long t;

		t = RobotController.getFPGATime();
		imu.sense();
		Logger.recordOutput("Timing/IMU_sense_ms", (RobotController.getFPGATime() - t) / 1000.0);

		t = RobotController.getFPGATime();
		drive.sense();
		Logger.recordOutput("Timing/Drive_sense_ms", (RobotController.getFPGATime() - t) / 1000.0);

		t = RobotController.getFPGATime();
		intake.sense();
		Logger.recordOutput("Timing/Intake_sense_ms", (RobotController.getFPGATime() - t) / 1000.0);

		t = RobotController.getFPGATime();
		shooter.sense();
		Logger.recordOutput("Timing/Shooter_sense_ms", (RobotController.getFPGATime() - t) / 1000.0);

		t = RobotController.getFPGATime();
		indexer.sense();
		Logger.recordOutput("Timing/Indexer_sense_ms", (RobotController.getFPGATime() - t) / 1000.0);

		t = RobotController.getFPGATime();
		vision.sense();
		Logger.recordOutput("Timing/Vision_sense_ms", (RobotController.getFPGATime() - t) / 1000.0);

		t = RobotController.getFPGATime();
		leds.sense();
		Logger.recordOutput("Timing/LEDS_sense_ms", (RobotController.getFPGATime() - t) / 1000.0);

		t = RobotController.getFPGATime();
		CommandScheduler.getInstance().run();
		Logger.recordOutput("Timing/CommandScheduler_ms", (RobotController.getFPGATime() - t) / 1000.0);

		autoChooser.periodic();
		if (DriverStation.isDisabled()) {
			leds.control = leds.DISABLED_ANIMATION;
			autonomousCommand = autoChooser.selectedCommand();
		} else {
			leds.control = shooter.onTarget ? leds.SHOOTING_ANIMATION : leds.IDLE_ANIMATION;
		}

		updateMatchPeriod();

		t = RobotController.getFPGATime();
		// drive.actuate();
		Logger.recordOutput("Timing/Drive_actuate_ms", (RobotController.getFPGATime() - t) / 1000.0);

		t = RobotController.getFPGATime();
		// intake.actuate();
		Logger.recordOutput("Timing/Intake_actuate_ms", (RobotController.getFPGATime() - t) / 1000.0);

		t = RobotController.getFPGATime();
		shooter.actuate();
		Logger.recordOutput("Timing/Shooter_actuate_ms", (RobotController.getFPGATime() - t) / 1000.0);

		t = RobotController.getFPGATime();
		indexer.actuate();
		Logger.recordOutput("Timing/Indexer_actuate_ms", (RobotController.getFPGATime() - t) / 1000.0);

		t = RobotController.getFPGATime();
		leds.actuate();
		Logger.recordOutput("Timing/LEDS_actuate_ms", (RobotController.getFPGATime() - t) / 1000.0);
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
			allianceHubStatus = "#000000";
		} else if (DriverStation.isAutonomous()) {
			currentPeriod = "AUTO";
			periodTimeRemaining = matchTime;
			allianceHubStatus = "#FF00FF";
		} else if (DriverStation.isTeleop()) {
			boolean weWonAuto = (isRed() && redWonAuto()) || (!isRed() && !redWonAuto());

			if (matchTime > 130) {
				currentPeriod = "TRANSITION";
				periodTimeRemaining = matchTime - 130;
				allianceHubStatus = weWonAuto ? "#FF0000" : "#00FF00";
			} else if (matchTime > 105) {
				currentPeriod = "SHIFT 1";
				periodTimeRemaining = matchTime - 105;
				// Odd shift: auto winner's hub inactive
				allianceHubStatus = weWonAuto ? "#FF0000" : "#00FF00";
			} else if (matchTime > 80) {
				currentPeriod = "SHIFT 2";
				periodTimeRemaining = matchTime - 80;
				// Even shift: auto winner's hub active
				allianceHubStatus = weWonAuto ? "#00FF00" : "#FF0000";
			} else if (matchTime > 55) {
				currentPeriod = "SHIFT 3";
				periodTimeRemaining = matchTime - 55;
				allianceHubStatus = weWonAuto ? "#FF0000" : "#00FF00";
			} else if (matchTime > 30) {
				currentPeriod = "SHIFT 4";
				periodTimeRemaining = matchTime - 30;
				allianceHubStatus = weWonAuto ? "#00FF00" : "#FF0000";
			} else {
				currentPeriod = "END GAME";
				periodTimeRemaining = matchTime;
				allianceHubStatus = "#00FFFF";
			}
		} else {
			currentPeriod = "TEST";
			periodTimeRemaining = 0;
			allianceHubStatus = "#000000";
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
