package frc.robot.subsystem.controls;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxControls implements Controls {
	public static CommandXboxController xbox = new CommandXboxController(1);
	public static CommandJoystick thirdJoystick = new CommandJoystick(2);
	public static CommandJoystick driverStation = new CommandJoystick(5);

	public Trigger sysidQuasiForward() {
		return xbox.y();
	}

	public Trigger sysidQuasiBackward() {
		return xbox.a();
	}

	public Trigger sysidDynaForward() {
		return xbox.x();
	}

	public Trigger sysidDynaBackward() {
		return xbox.b();
	}

	public double leftX() {
		return -xbox.getLeftX();
	}

	public double leftY() {
		return -xbox.getLeftY();
	}

	public double rightX() {
		return -xbox.getRightX();
	}

	public double rightY() {
		return -xbox.getRightY();
	}

	public Trigger shoot() {
		return xbox.rightBumper();
	}

	public Trigger intake() {
		return xbox.leftBumper();
	}

	public Trigger pov(int angle) {
		return xbox.pov(angle);
	}

	public Trigger rightPov(int angle) {
		return xbox.pov(angle);
	}

	public double thirdX() {
		return -thirdJoystick.getX();
	}

	public double thirdY() {
		return -thirdJoystick.getY();
	}

	public Trigger intakeDown() {
		return driverStation.button(12);
	}

	public Trigger intakeUp() {
		return driverStation.button(11);
	}

	public Trigger intakeReverse() {
		return driverStation.button(10);
	}

	public Trigger indexerReverse() {
		return driverStation.button(9);
	}

	public Trigger climberForward() {
		return driverStation.button(6);
	}

	public Trigger climberUp() {
		return driverStation.button(8);
	}

	public Trigger climberDown() {
		return driverStation.button(7);
	}

	public Trigger eStopShooter() {
		return driverStation.button(15);
	}

	public Trigger eStopAll() {
		return driverStation.button(14);
	}

	public Trigger hoodZero() {
		return driverStation.button(3);
	}
}
