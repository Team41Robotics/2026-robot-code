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

	public Trigger shoot();

	public Trigger intake();

	public Trigger pov(int angle);

	public Trigger rightPov(int angle);

	public double thirdX();

	public double thirdY();

	public Trigger intakeDown();

	public Trigger intakeUp();

	public Trigger intakeReverse();

	public Trigger indexerReverse();

	public Trigger climberForward();

	public Trigger climberUp();

	public Trigger climberDown();

	public Trigger eStopShooter();

	public Trigger eStopAll();

	public Trigger hoodZero();

	public Trigger passToOwnSide();
}
