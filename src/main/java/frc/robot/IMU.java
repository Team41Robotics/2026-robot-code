package frc.robot;

import static java.lang.Math.PI;

import com.studica.frc.AHRS;

public class IMU {
	private AHRS imu;

	public IMU() {
		imu = new AHRS(AHRS.NavXComType.kMXP_SPI);
	}

	public double yaw() {
		return -imu.getAngle() / 180 * PI;
	}

	public void zeroYaw() {
		imu.zeroYaw();
	}

	public double getAngularVelocity() {
		return imu.getRate();
	}
}