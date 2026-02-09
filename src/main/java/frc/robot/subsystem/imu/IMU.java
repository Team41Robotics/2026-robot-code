package frc.robot.subsystem.imu;

import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IMU extends SubsystemBase {
	public IMUHW hw = new IMUHW();
	public IMUInputsAutoLogged inputs;

	public double yaw;

	public void init() {
		inputs = new IMUInputsAutoLogged();
		hw.init();
		hw.sense(inputs);
	}

	public void sense() {
		hw.sense(inputs);

		yaw = inputs.yaw;

		Logger.processInputs("/IMU", inputs);
	}

	public void periodic_() {
		sense();
	}
}
