package frc.robot.subsystem.imu;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IMU extends SubsystemBase {
	public IMUNavXHW hw = new IMUNavXHW();
	public IMUInputsAutoLogged inputs;

	public double yaw;

	public void init() {
		inputs = new IMUInputsAutoLogged();
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);

		yaw = inputs.yaw;

		Logger.processInputs("/IMU", inputs);
	}
}
