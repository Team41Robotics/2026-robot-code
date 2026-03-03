package frc.robot.subsystem.imu;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IMUInputs {
	public boolean isConnected;
	public boolean isCalibrating;
	public double yawRadians;
	public double pitchRadians;
	public double rollRadians;
}
