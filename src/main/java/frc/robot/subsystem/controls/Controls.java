package frc.robot.subsystem.controls;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controls {

	public Trigger sysidQuasiForward();

	public Trigger sysidQuasiBackward();

	public Trigger sysidDynaForward();

	public Trigger sysidDynaBackward();

	public double leftX();

	public double leftY();

	public double rightX();

	public double rightY();
}
