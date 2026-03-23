package frc.robot.subsystem.controls;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickControls implements Controls {
	public static CommandJoystick leftJoystick = new CommandJoystick(3);
	public static CommandJoystick rightJoystick = new CommandJoystick(4);
	public static CommandJoystick thirdJoystick = new CommandJoystick(5);
	public static CommandJoystick driverStation = new CommandJoystick(2);

	public Trigger sysidQuasiForward() {
		return leftJoystick.button(1);
	}

	public Trigger sysidQuasiBackward() {
		return leftJoystick.button(2);
	}

	public Trigger sysidDynaForward() {
		return leftJoystick.button(3);
	}

	public Trigger sysidDynaBackward() {
		return leftJoystick.button(4);
	}

	public double leftX() {
		return -leftJoystick.getX();
	}

	public double leftY() {
		return -leftJoystick.getY();
	}

	public double rightX() {
		return -rightJoystick.getX();
	}

	public double rightY() {
		return -rightJoystick.getY();
	}

	public Trigger shoot() {
		return rightJoystick.button(1);
	}

	public Trigger intake() {
		return leftJoystick.button(1);
	}

	public Trigger pov(int angle) {
		return leftJoystick.pov(angle);
	}

	public Trigger rightPov(int angle) {
		return rightJoystick.pov(angle);
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

	public Trigger eStopShooter() {
		return driverStation.button(15);
	}

	public Trigger eStopAll() {
		return driverStation.button(14);
	}

	public Trigger hoodZero() {
		return rightJoystick.button(2);
	}

	public Trigger passToOwnSide() {
		return driverStation.button(2);
	}

	public Trigger driveLock() {
		return leftJoystick.button(2);
	}
}
