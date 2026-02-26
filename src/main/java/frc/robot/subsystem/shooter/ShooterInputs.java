package frc.robot.subsystem.shooter;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ShooterInputs {
	// Turret
	public double turretPos; // radians (mechanism space)
	public double turretVel; // rad/s
	public double turretVoltage;
	public double turretCurrent;
	public double turretBusVoltage;
	public double turretBusCurrent;

	// Hood
	public double hoodPos; // radians (mechanism space)
	public double hoodVel; // rad/s
	public double hoodVoltage;
	public double hoodCurrent;
	public double hoodBusVoltage;
	public double hoodBusCurrent;

	// Flywheel (leader only — follower mirrors)
	public double flywheelVel; // rad/s (mechanism space)
	public double flywheelVoltage;
	public double flywheelCurrent;
	public double flywheelBusVoltage;
	public double flywheelBusCurrent;
}
