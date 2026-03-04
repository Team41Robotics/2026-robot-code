package frc.robot.subsystem.controls;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxControls implements Controls {
	public static CommandXboxController xbox = new CommandXboxController(1);
	public static CommandJoystick driverStation = new CommandJoystick(2);

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
}
