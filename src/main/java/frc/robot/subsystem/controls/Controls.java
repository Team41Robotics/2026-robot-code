package frc.robot.subsystem.controls;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controls {

	public Trigger sysid_quasi_forward();

	public Trigger sysid_quasi_backward();

	public Trigger sysid_dyna_forward();

	public Trigger sysid_dyna_backward();

	public double leftX();

	public double leftY();

	public double rightX();

	public double rightY();
}
