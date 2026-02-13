package frc.robot.subsystem.controls;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickControls implements Controls {
	public static CommandJoystick left_js = new CommandJoystick(3);
	public static CommandJoystick right_js = new CommandJoystick(4);
	public static CommandJoystick ds = new CommandJoystick(2);

	public Trigger sysidQuasiForward() {
		return left_js.button(1);
	}

	public Trigger sysidQuasiBackward() {
		return left_js.button(2);
	}

	public Trigger sysidDynaForward() {
		return left_js.button(3);
	}

	public Trigger sysidDynaBackward() {
		return left_js.button(4);
	}

	public double leftX() {
		return left_js.getX();
	}

	public double leftY() {
		return left_js.getY();
	}

	public double rightX() {
		return -right_js.getX();
	}

	public double rightY() {
		return -right_js.getY();
	}
}
