package frc.robot.subsystem.imu;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Robot;

public class IMUPigeonHW {
	public Pigeon2 imu;

	public void init() {
		if (!Robot.isReal()) return;

		imu = new Pigeon2(2);
		imu.reset();
	}

	public void sense(IMUInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.isConnected = imu.isConnected();
		inputs.isCalibrating = false;
		inputs.yaw = imu.getRotation2d().getRadians();
	}
}
