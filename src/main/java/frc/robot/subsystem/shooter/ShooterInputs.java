package frc.robot.subsystem.shooter;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ShooterInputs {
	public double turretPosRadians;
	public double turretVelRadiansPerSec;
	public double turretVoltageVolts;
	public double turretCurrentAmps;
	public double turretBusVoltageVolts;
	public double turretBusCurrentAmps;
	public double turretTsSec;

	public double hoodPosRadians;
	public double hoodVelRadiansPerSec;
	public double hoodVoltageVolts;
	public double hoodCurrentAmps;
	public double hoodBusVoltageVolts;
	public double hoodBusCurrentAmps;

	public double hoodTsSec;

	public double flywheelVelocityRPM;
	public double flywheelVoltageVolts;
	public double flywheelCurrentAmps;
	public double flywheelBusVoltageVolts;
	public double flywheelBusCurrentAmps;
	public double flywheelTsSec;
}
