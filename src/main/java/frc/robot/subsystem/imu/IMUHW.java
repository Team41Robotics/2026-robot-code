package frc.robot.subsystem.imu;

import static edu.wpi.first.math.MathUtil.*;
import static java.lang.Math.*;

import com.studica.frc.AHRS;
import frc.robot.Robot;

public class IMUHW {
	public AHRS imu;

	public void init() {
		if (!Robot.isReal()) return;

		imu = new AHRS(AHRS.NavXComType.kMXP_SPI);
		imu.zeroYaw();
	}

	public void sense(IMUInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.connected = imu.isConnected();
		inputs.yaw = -imu.getAngle() / 180 * PI;
	}
}
