package frc.robot.subsystem.shooter;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ShooterInputs {
	// Turret
	public double turretPosRadians;
	public double turretVelRadiansPerSec;
	public double turretVoltageVolts;
	public double turretCurrentAmps;
	public double turretBusVoltageVolts;
	public double turretBusCurrentAmps;

	// Hood
	public double hoodPosRadians;
	public double hoodVelRadiansPerSec;
	public double hoodVoltageVolts;
	public double hoodCurrentAmps;
	public double hoodBusVoltageVolts;
	public double hoodBusCurrentAmps;
	public boolean isHoodLimitSwitchOn;

	// Flywheel (leader only — follower mirrors)
	public double flywheelVelRadiansPerSec;
	public double flywheelVoltageVolts;
	public double flywheelCurrentAmps;
	public double flywheelBusVoltageVolts;
	public double flywheelBusCurrentAmps;
}
