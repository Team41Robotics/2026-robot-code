package frc.robot.subsystem.imu;

import static frc.robot.RobotContainer.driveBus;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Robot;

public class IMUPigeonHW {
	public Pigeon2 imu;

	public void init() {
		if (!Robot.isReal()) return;

		imu = new Pigeon2(23, driveBus);
		imu.reset();
	}

	public void sense(IMUInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.isConnected = imu.isConnected();
		inputs.isCalibrating = false;
		inputs.yawRadians = imu.getRotation2d().getRadians();
	}
}
