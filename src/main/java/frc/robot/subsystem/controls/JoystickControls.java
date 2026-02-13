package frc.robot.subsystem.controls;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickControls implements Controls {
	public static CommandJoystick leftJoystick = new CommandJoystick(3);
	public static CommandJoystick rightJoystick = new CommandJoystick(4);
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
		return leftJoystick.getX();
	}

	public double leftY() {
		return leftJoystick.getY();
	}

	public double rightX() {
		return -rightJoystick.getX();
	}

	public double rightY() {
		return -rightJoystick.getY();
	}
}
