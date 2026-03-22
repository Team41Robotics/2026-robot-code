package frc.robot.commands;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;

public class PreMatchCheck extends Command {
	public static boolean allPassed = false;

	public static Alert batteryOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert batteryFail = new Alert("PreMatch", "", AlertType.kError);

	public static Alert imuConnOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert imuConnFail = new Alert("PreMatch", "", AlertType.kError);

	public static Alert imuCalOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert imuCalFail = new Alert("PreMatch", "", AlertType.kError);

	public static Alert[] swerveOk = new Alert[8];
	public static Alert[] swerveFail = new Alert[8];

	public static Alert[] visionOk = new Alert[2];
	public static Alert[] visionFail = new Alert[2];

	public static Alert flywheelOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert flywheelFail = new Alert("PreMatch", "", AlertType.kError);

	public static Alert intakeExtOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert intakeExtFail = new Alert("PreMatch", "", AlertType.kError);

	public static Alert intakeRollerOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert intakeRollerFail = new Alert("PreMatch", "", AlertType.kError);

	public static Alert indexerRollersOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert indexerRollersFail = new Alert("PreMatch", "", AlertType.kError);

	public static Alert indexerElevOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert indexerElevFail = new Alert("PreMatch", "", AlertType.kError);

	public static Alert ledsOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert ledsFail = new Alert("PreMatch", "", AlertType.kError);

	public static Alert summaryOk = new Alert("PreMatch", "", AlertType.kInfo);
	public static Alert summaryFail = new Alert("PreMatch", "", AlertType.kError);

	static {
		for (int i = 0; i < 8; i++) {
			swerveOk[i] = new Alert("PreMatch", "", AlertType.kInfo);
			swerveFail[i] = new Alert("PreMatch", "", AlertType.kError);
		}
		for (int i = 0; i < 2; i++) {
			visionOk[i] = new Alert("PreMatch", "", AlertType.kInfo);
			visionFail[i] = new Alert("PreMatch", "", AlertType.kError);
		}
	}

	static boolean check(Alert ok, Alert fail, String name, boolean passed) {
		ok.setText(name + " OK");
		fail.setText(name + " FAIL");
		ok.set(passed);
		fail.set(!passed);
		return passed;
	}

	@Override
	public void execute() {
		allPassed = true;

		double busV = drive.modules[0].inputs.driveBusVoltageVolts;
		allPassed &= check(batteryOk, batteryFail, "Battery " + String.format("%.1fV", busV), busV > 11.5);

		allPassed &= check(imuConnOk, imuConnFail, "IMU Connected", imu.inputs.isConnected);
		allPassed &= check(imuCalOk, imuCalFail, "IMU Calibrated", !imu.inputs.isCalibrating);

		String[] moduleNames = {"NW", "NE", "SW", "SE"};
		for (int i = 0; i < 4; i++) {
			allPassed &= check(
					swerveOk[i * 2],
					swerveFail[i * 2],
					"Swerve " + moduleNames[i] + " Drive",
					drive.modules[i].inputs.driveBusVoltageVolts > 0);
			allPassed &= check(
					swerveOk[i * 2 + 1],
					swerveFail[i * 2 + 1],
					"Swerve " + moduleNames[i] + " Turn",
					drive.modules[i].inputs.turnBusVoltageVolts > 0);
		}

		for (int i = 0; i < vision.nCams; i++) {
			allPassed &=
					check(visionOk[i], visionFail[i], "Vision " + vision.cameras[i].name, vision.inputs[i].isConnected);
		}

		allPassed &= check(flywheelOk, flywheelFail, "Shooter Flywheel", shooter.inputs.flywheelBusVoltageVolts > 0);

		allPassed &= check(intakeExtOk, intakeExtFail, "Intake Extension", intake.inputs.extensionBusVoltageVolts > 0);
		allPassed &= check(intakeRollerOk, intakeRollerFail, "Intake Roller", intake.inputs.intakeBusVoltageVolts > 0);

		allPassed &= check(
				indexerRollersOk, indexerRollersFail, "Indexer Rollers", indexer.inputs.rollersBusVoltageVolts > 0);
		allPassed &=
				check(indexerElevOk, indexerElevFail, "Indexer Elevator", indexer.inputs.elevatorBusVoltageVolts > 0);

		allPassed &= check(ledsOk, ledsFail, "LEDs", leds.inputs.busVoltageVolts > 0);

		check(summaryOk, summaryFail, "ALL SYSTEMS", allPassed);
	}
}
