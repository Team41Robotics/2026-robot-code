package frc.robot.subsystem.drive;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SwerveInputs {
	public double drivePosMeters;
	public double driveVelMetersPerSec;
	public double driveVoltageVolts;
	public double driveBusVoltageVolts;
	public double driveCurrentAmps;
	public double driveBusCurrentAmps;

	public double turnPosRadians;
	public double turnVelRadiansPerSec;
	public double turnVoltageVolts;
	public double turnBusVoltageVolts;
	public double turnCurrentAmps;
	public double turnBusCurrentAmps;

	public double turnAbsPosRadians;
}
