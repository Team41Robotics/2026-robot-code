package frc.robot.commands;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class MatchAlerts extends Command {
	public static final double STALE_Sec = 0.300;
	public static final double MIN_LOG_TS_Sec = 0.050;

	// Track previous state + timestamps and last-change time, indexed by CAN ID
	public static long[] prevHash = new long[53];
	public static double[] prevTsSec = new double[53];
	public static double[] lastUpdateTimeSec = new double[53];
	public static double[] downStartTimeSec = new double[53];

	// Cumulative downtime tracking (seconds)
	public static double[] totalDownSec = new double[53];
	public static double[] totalEverDownSec = new double[53];

	static {
		for (int i = 0; i < 53; i++) {
			prevHash[i] = 0;
			prevTsSec[i] = Double.NaN;
			lastUpdateTimeSec[i] = 0;
			downStartTimeSec[i] = Double.NaN;
			totalDownSec[i] = 0;
			totalEverDownSec[i] = 0;
		}
	}

	// Alerts ordered by CAN ID
	public static Alert ledsAlert = new Alert("MatchAlerts", "", AlertType.kWarning); // 6
	public static Alert swerveSETurn = new Alert("MatchAlerts", "", AlertType.kWarning); // 11
	public static Alert swerveSEEncoder = new Alert("MatchAlerts", "", AlertType.kWarning); // 12
	public static Alert swerveSEDrive = new Alert("MatchAlerts", "", AlertType.kWarning); // 13
	public static Alert swerveSWDrive = new Alert("MatchAlerts", "", AlertType.kWarning); // 14
	public static Alert swerveSWEncoder = new Alert("MatchAlerts", "", AlertType.kWarning); // 15
	public static Alert swerveSWTurn = new Alert("MatchAlerts", "", AlertType.kWarning); // 16
	public static Alert swerveNWTurn = new Alert("MatchAlerts", "", AlertType.kWarning); // 17
	public static Alert swerveNWEncoder = new Alert("MatchAlerts", "", AlertType.kWarning); // 18
	public static Alert swerveNWDrive = new Alert("MatchAlerts", "", AlertType.kWarning); // 19
	public static Alert swerveNEDrive = new Alert("MatchAlerts", "", AlertType.kWarning); // 20
	public static Alert swerveNEEncoder = new Alert("MatchAlerts", "", AlertType.kWarning); // 21
	public static Alert swerveNETurn = new Alert("MatchAlerts", "", AlertType.kWarning); // 22
	public static Alert imuAlert = new Alert("MatchAlerts", "", AlertType.kWarning); // 23
	public static Alert intakeRollerAlert = new Alert("MatchAlerts", "", AlertType.kWarning); // 31
	public static Alert intakeEncoderAlert = new Alert("MatchAlerts", "", AlertType.kWarning); // 32
	public static Alert intakeJointAlert = new Alert("MatchAlerts", "", AlertType.kWarning); // 33
	public static Alert indexerElevAlert = new Alert("MatchAlerts", "", AlertType.kWarning); // 41
	public static Alert turretAlert = new Alert("MatchAlerts", "", AlertType.kWarning); // 42
	public static Alert indexerSpinAlert = new Alert("MatchAlerts", "", AlertType.kWarning); // 43
	public static Alert backvatorAlert = new Alert("MatchAlerts", "", AlertType.kWarning); // 44
	public static Alert flywheelAlert = new Alert("MatchAlerts", "", AlertType.kWarning); // 51
	public static Alert hoodAlert = new Alert("MatchAlerts", "", AlertType.kWarning); // 52

	// Persistent error alerts: device has disconnected at least once during the enabled period
	public static Alert ledsError = new Alert("MatchAlerts", "", AlertType.kError); // 6
	public static Alert swerveSETurnError = new Alert("MatchAlerts", "", AlertType.kError); // 11
	public static Alert swerveSEEncoderError = new Alert("MatchAlerts", "", AlertType.kError); // 12
	public static Alert swerveSEDriveError = new Alert("MatchAlerts", "", AlertType.kError); // 13
	public static Alert swerveSWDriveError = new Alert("MatchAlerts", "", AlertType.kError); // 14
	public static Alert swerveSWEncoderError = new Alert("MatchAlerts", "", AlertType.kError); // 15
	public static Alert swerveSWTurnError = new Alert("MatchAlerts", "", AlertType.kError); // 16
	public static Alert swerveNWTurnError = new Alert("MatchAlerts", "", AlertType.kError); // 17
	public static Alert swerveNWEncoderError = new Alert("MatchAlerts", "", AlertType.kError); // 18
	public static Alert swerveNWDriveError = new Alert("MatchAlerts", "", AlertType.kError); // 19
	public static Alert swerveNEDriveError = new Alert("MatchAlerts", "", AlertType.kError); // 20
	public static Alert swerveNEEncoderError = new Alert("MatchAlerts", "", AlertType.kError); // 21
	public static Alert swerveNETurnError = new Alert("MatchAlerts", "", AlertType.kError); // 22
	public static Alert imuError = new Alert("MatchAlerts", "", AlertType.kError); // 23
	public static Alert intakeRollerError = new Alert("MatchAlerts", "", AlertType.kError); // 31
	public static Alert intakeEncoderError = new Alert("MatchAlerts", "", AlertType.kError); // 32
	public static Alert intakeJointError = new Alert("MatchAlerts", "", AlertType.kError); // 33
	public static Alert indexerElevError = new Alert("MatchAlerts", "", AlertType.kError); // 41
	public static Alert turretError = new Alert("MatchAlerts", "", AlertType.kError); // 42
	public static Alert indexerSpinError = new Alert("MatchAlerts", "", AlertType.kError); // 43
	public static Alert backvatorError = new Alert("MatchAlerts", "", AlertType.kError); // 44
	public static Alert flywheelError = new Alert("MatchAlerts", "", AlertType.kError); // 51
	public static Alert hoodError = new Alert("MatchAlerts", "", AlertType.kError); // 52

	// Track whether a device has disconnected at least once since last reset (indexed by CAN ID)
	public static boolean[] hasDisconnectedOnce = new boolean[53];

	/**
	 * Unified device disconnect checker.
	 *
	 * <p>A device is considered "up" when either:
	 *
	 * <ul>
	 *   <li>any monitored value changes (state fingerprint changes), or
	 *   <li>any CTRE timestamp increases (phoenix StatusSignal timestamp changes)
	 * </ul>
	 *
	 * <p>When no change is observed for {@link #STALE_Sec}, the warning alert is set ("down rn").
	 * Once a device has ever been down, the error alert is latched for the rest of the program run.
	 * Both alerts include cumulative downtime.
	 */
	static void check(
			Alert warnAlert,
			Alert errorAlert,
			String name,
			int canId,
			double[] values,
			double[] tsSec) {
		double now = Timer.getTimestamp();

		long h = 0xcbf29ce484222325L; // FNV-1a 64-bit offset
		if (values != null) {
			for (double v : values) {
				long bits = Double.doubleToLongBits(v);
				h ^= bits;
				h *= 0x100000001b3L;
			}
		}
		double bestTs = Double.NaN;
		if (tsSec != null) {
			for (double t : tsSec) {
				if (Double.isFinite(t) && t > MIN_LOG_TS_Sec) {
					if (!Double.isFinite(bestTs) || t > bestTs) bestTs = t;
				}
			}
			for (double t : tsSec) {
				long bits = Double.doubleToLongBits(t);
				h ^= bits;
				h *= 0x100000001b3L;
			}
		}

		boolean updated = false;
		if (h != prevHash[canId]) {
			prevHash[canId] = h;
			updated = true;
		}
		if (Double.isFinite(bestTs) && (Double.isNaN(prevTsSec[canId]) || bestTs != prevTsSec[canId])) {
			prevTsSec[canId] = bestTs;
			updated = true;
		}

		if (updated) {
			lastUpdateTimeSec[canId] = now;
		}

		boolean downNow = (now - lastUpdateTimeSec[canId]) > STALE_Sec;
		if (downNow) {
			hasDisconnectedOnce[canId] = true;
			if (!Double.isFinite(downStartTimeSec[canId])) {
				downStartTimeSec[canId] = now;
			}
		} else {
			if (Double.isFinite(downStartTimeSec[canId])) {
				double dt = now - downStartTimeSec[canId];
				totalDownSec[canId] += dt;
				totalEverDownSec[canId] += dt;
				downStartTimeSec[canId] = Double.NaN;
			}
			warnAlert.set(false);
		}

		if (downNow) {
			double currentDown = now - downStartTimeSec[canId];
			warnAlert.setText(
					name
							+ " is DOWN"
							+ String.format(" (down %.2fs total)", totalDownSec[canId] + currentDown));
			warnAlert.set(true);
		}

		if (hasDisconnectedOnce[canId]) {
			double everDown = totalEverDownSec[canId];
			if (downNow) everDown += (now - downStartTimeSec[canId]);
			errorAlert.setText(
					name
							+ " has disconnected"
							+ String.format(" (down %.2fs total)", everDown));
			errorAlert.set(true);
		} else {
			errorAlert.set(false);
		}
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	@Override
	public void execute() {
		// Persist alerts through disabled. While disabled, we still run checks; if inputs
		// stop updating in disabled, it will accumulate down-time (which is usually fine
		// since it reflects real bus silence). If you'd rather pause timers in disabled,
		// we can gate the accumulation on DriverStation.isEnabled().

		// CAN ID 6
		check(
				ledsAlert,
				ledsError,
				"LEDs CANdle (6)",
				6,
				new double[] {
					leds.inputs.busVoltageVolts,
					leds.inputs.outputCurrentAmps,
					leds.inputs.fiveVRailVoltageVolts,
					leds.inputs.boardTempCelsius,
					leds.inputs.vBatModulation
				},
				new double[] {leds.inputs.tsSec});
		// CAN ID 11-13: SE module
		check(
				swerveSETurn,
				swerveSETurnError,
				"Swerve SE Turn (11)",
				11,
				new double[] {
					drive.modules[3].inputs.turnBusVoltageVolts,
					drive.modules[3].inputs.turnBusCurrentAmps,
					drive.modules[3].inputs.turnVoltageVolts,
					drive.modules[3].inputs.turnCurrentAmps,
					drive.modules[3].inputs.turnPosRadians,
					drive.modules[3].inputs.turnVelRadiansPerSec
				},
				new double[] {drive.modules[3].inputs.turnTsSec});
		check(
				swerveSEEncoder,
				swerveSEEncoderError,
				"Swerve SE CANcoder (12)",
				12,
				new double[] {drive.modules[3].inputs.turnAbsBusVoltageVolts, drive.modules[3].inputs.turnAbsPosRadians},
				new double[] {drive.modules[3].inputs.turnAbsTsSec});
		check(
				swerveSEDrive,
				swerveSEDriveError,
				"Swerve SE Drive (13)",
				13,
				new double[] {
					drive.modules[3].inputs.driveBusVoltageVolts,
					drive.modules[3].inputs.driveBusCurrentAmps,
					drive.modules[3].inputs.driveVoltageVolts,
					drive.modules[3].inputs.driveCurrentAmps,
					drive.modules[3].inputs.drivePosMeters,
					drive.modules[3].inputs.driveVelMetersPerSec
				},
				new double[] {drive.modules[3].inputs.driveTsSec});
		// CAN ID 14-16: SW module
		check(
				swerveSWDrive,
				swerveSWDriveError,
				"Swerve SW Drive (14)",
				14,
				new double[] {
					drive.modules[2].inputs.driveBusVoltageVolts,
					drive.modules[2].inputs.driveBusCurrentAmps,
					drive.modules[2].inputs.driveVoltageVolts,
					drive.modules[2].inputs.driveCurrentAmps,
					drive.modules[2].inputs.drivePosMeters,
					drive.modules[2].inputs.driveVelMetersPerSec
				},
				new double[] {drive.modules[2].inputs.driveTsSec});
		check(
				swerveSWEncoder,
				swerveSWEncoderError,
				"Swerve SW CANcoder (15)",
				15,
				new double[] {drive.modules[2].inputs.turnAbsBusVoltageVolts, drive.modules[2].inputs.turnAbsPosRadians},
				new double[] {drive.modules[2].inputs.turnAbsTsSec});
		check(
				swerveSWTurn,
				swerveSWTurnError,
				"Swerve SW Turn (16)",
				16,
				new double[] {
					drive.modules[2].inputs.turnBusVoltageVolts,
					drive.modules[2].inputs.turnBusCurrentAmps,
					drive.modules[2].inputs.turnVoltageVolts,
					drive.modules[2].inputs.turnCurrentAmps,
					drive.modules[2].inputs.turnPosRadians,
					drive.modules[2].inputs.turnVelRadiansPerSec
				},
				new double[] {drive.modules[2].inputs.turnTsSec});
		// CAN ID 17-19: NW module
		check(
				swerveNWTurn,
				swerveNWTurnError,
				"Swerve NW Turn (17)",
				17,
				new double[] {
					drive.modules[0].inputs.turnBusVoltageVolts,
					drive.modules[0].inputs.turnBusCurrentAmps,
					drive.modules[0].inputs.turnVoltageVolts,
					drive.modules[0].inputs.turnCurrentAmps,
					drive.modules[0].inputs.turnPosRadians,
					drive.modules[0].inputs.turnVelRadiansPerSec
				},
				new double[] {drive.modules[0].inputs.turnTsSec});
		check(
				swerveNWEncoder,
				swerveNWEncoderError,
				"Swerve NW CANcoder (18)",
				18,
				new double[] {drive.modules[0].inputs.turnAbsBusVoltageVolts, drive.modules[0].inputs.turnAbsPosRadians},
				new double[] {drive.modules[0].inputs.turnAbsTsSec});
		check(
				swerveNWDrive,
				swerveNWDriveError,
				"Swerve NW Drive (19)",
				19,
				new double[] {
					drive.modules[0].inputs.driveBusVoltageVolts,
					drive.modules[0].inputs.driveBusCurrentAmps,
					drive.modules[0].inputs.driveVoltageVolts,
					drive.modules[0].inputs.driveCurrentAmps,
					drive.modules[0].inputs.drivePosMeters,
					drive.modules[0].inputs.driveVelMetersPerSec
				},
				new double[] {drive.modules[0].inputs.driveTsSec});
		// CAN ID 20-22: NE module
		check(
				swerveNEDrive,
				swerveNEDriveError,
				"Swerve NE Drive (20)",
				20,
				new double[] {
					drive.modules[1].inputs.driveBusVoltageVolts,
					drive.modules[1].inputs.driveBusCurrentAmps,
					drive.modules[1].inputs.driveVoltageVolts,
					drive.modules[1].inputs.driveCurrentAmps,
					drive.modules[1].inputs.drivePosMeters,
					drive.modules[1].inputs.driveVelMetersPerSec
				},
				new double[] {drive.modules[1].inputs.driveTsSec});
		check(
				swerveNEEncoder,
				swerveNEEncoderError,
				"Swerve NE CANcoder (21)",
				21,
				new double[] {drive.modules[1].inputs.turnAbsBusVoltageVolts, drive.modules[1].inputs.turnAbsPosRadians},
				new double[] {drive.modules[1].inputs.turnAbsTsSec});
		check(
				swerveNETurn,
				swerveNETurnError,
				"Swerve NE Turn (22)",
				22,
				new double[] {
					drive.modules[1].inputs.turnBusVoltageVolts,
					drive.modules[1].inputs.turnBusCurrentAmps,
					drive.modules[1].inputs.turnVoltageVolts,
					drive.modules[1].inputs.turnCurrentAmps,
					drive.modules[1].inputs.turnPosRadians,
					drive.modules[1].inputs.turnVelRadiansPerSec
				},
				new double[] {drive.modules[1].inputs.turnTsSec});
		// CAN ID 23
		check(
				imuAlert,
				imuError,
				"IMU Pigeon2 (23)",
				23,
				new double[] {imu.inputs.yawRadians, imu.inputs.pitchRadians, imu.inputs.rollRadians},
				new double[] {imu.inputs.yawTsSec});
		// CAN ID 31-33: Intake
		check(
				intakeRollerAlert,
				intakeRollerError,
				"Intake Roller (31)",
				31,
				new double[] {
					intake.inputs.intakeBusVoltageVolts,
					intake.inputs.intakeVoltageVolts,
					intake.inputs.intakeCurrentAmps,
					intake.inputs.intakeVelocityRPM
				},
				new double[] {intake.inputs.intakeLastGoodTimeSec});
		check(
				intakeEncoderAlert,
				intakeEncoderError,
				"Intake CANcoder (32)",
				32,
				new double[] {intake.inputs.jointPosRadians},
				new double[] {intake.inputs.jointAbsTsSec});
		check(
				intakeJointAlert,
				intakeJointError,
				"Intake Joint (33)",
				33,
				new double[] {
					intake.inputs.jointBusVoltageVolts,
					intake.inputs.jointVoltageVolts,
					intake.inputs.jointCurrentAmps,
					intake.inputs.jointVelRadiansPerSec,
					intake.inputs.jointPosRadians
				},
				new double[] {intake.inputs.jointLastGoodTimeSec, intake.inputs.jointAbsTsSec});
		// CAN ID 41
		check(
				indexerElevAlert,
				indexerElevError,
				"Indexer Elevator (41)",
				41,
				new double[] {
					indexer.inputs.elevatorBusVoltageVolts,
					indexer.inputs.elevatorBusCurrentAmps,
					indexer.inputs.elevatorVoltageVolts,
					indexer.inputs.elevatorCurrentAmps,
					indexer.inputs.elevatorVelocityRPM
				},
				new double[] {indexer.inputs.elevatorTsSec});
		// CAN ID 42
		check(
				turretAlert,
				turretError,
				"Shooter Turret (42)",
				42,
				new double[] {
					shooter.inputs.turretBusVoltageVolts,
					shooter.inputs.turretBusCurrentAmps,
					shooter.inputs.turretVoltageVolts,
					shooter.inputs.turretCurrentAmps,
					shooter.inputs.turretPosRadians,
					shooter.inputs.turretVelRadiansPerSec
				},
				new double[] {shooter.inputs.turretTsSec});
		// CAN ID 43
		check(
				indexerSpinAlert,
				indexerSpinError,
				"Indexer Spin (43)",
				43,
				new double[] {
					indexer.inputs.spinBusVoltageVolts,
					indexer.inputs.spinBusCurrentAmps,
					indexer.inputs.spinVoltageVolts,
					indexer.inputs.spinCurrentAmps,
					indexer.inputs.spinVelocityRPM
				},
				new double[] {indexer.inputs.spinTsSec});
		// CAN ID 44
		check(
				backvatorAlert,
				backvatorError,
				"Indexer Backvator (44)",
				44,
				new double[] {
					indexer.inputs.backvatorVoltageVolts,
					indexer.inputs.backvatorCurrentAmps,
					indexer.inputs.backvatorVelocityRPM
				},
				new double[] {indexer.inputs.backvatorLastGoodTimeSec});
		// CAN ID 51
		check(
				flywheelAlert,
				flywheelError,
				"Shooter Flywheel (51)",
				51,
				new double[] {
					shooter.inputs.flywheelBusVoltageVolts,
					shooter.inputs.flywheelBusCurrentAmps,
					shooter.inputs.flywheelVoltageVolts,
					shooter.inputs.flywheelCurrentAmps,
					shooter.inputs.flywheelVelocityRPM
				},
				new double[] {shooter.inputs.flywheelTsSec});
		// CAN ID 52
		check(
				hoodAlert,
				hoodError,
				"Shooter Hood (52)",
				52,
				new double[] {
					shooter.inputs.hoodBusVoltageVolts,
					shooter.inputs.hoodBusCurrentAmps,
					shooter.inputs.hoodVoltageVolts,
					shooter.inputs.hoodCurrentAmps,
					shooter.inputs.hoodPosRadians,
					shooter.inputs.hoodVelRadiansPerSec
				},
				new double[] {shooter.inputs.hoodTsSec});
	}
}
