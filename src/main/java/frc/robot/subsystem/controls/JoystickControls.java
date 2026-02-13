package frc.robot.subsystem.controls;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickControls implements Controls {
	public static CommandJoystick left_js = new CommandJoystick(3); // FIXME. left joystick port
	public static CommandJoystick right_js = new CommandJoystick(4); // FIXME. right joystick port
	public static CommandJoystick ds = new CommandJoystick(2); // FIXME. driver station joystick port

	public Trigger sysid_quasi_forward() {
		return left_js.button(1);
	}

	public Trigger sysid_quasi_backward() {
		return left_js.button(2);
	}

	public Trigger sysid_dyna_forward() {
		return left_js.button(3);
	}

	public Trigger sysid_dyna_backward() {
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
