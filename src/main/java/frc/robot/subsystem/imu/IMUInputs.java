package frc.robot.subsystem.imu;

import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IMUInputs {
	public boolean isConnected;
	public boolean isCalibrating;
	public double yawRadians;
	public Rotation3d rotation3d = new Rotation3d();
}
