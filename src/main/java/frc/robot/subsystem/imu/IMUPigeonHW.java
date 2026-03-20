package frc.robot.subsystem.imu;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Robot;

public class IMUPigeonHW {
	public Pigeon2 imu;

	// Cached StatusSignals
	public StatusSignal<Angle> yawSignal;
	public StatusSignal<Angle> pitchSignal;
	public StatusSignal<Angle> rollSignal;
	public StatusSignal<AngularVelocity> yawRateSignal;

	public void init() {
		if (!Robot.isReal()) return;

		imu = new Pigeon2(23, driveBus);
		imu.reset();

		// Initialize cached signals
		yawSignal = imu.getYaw(false);
		pitchSignal = imu.getPitch(false);
		rollSignal = imu.getRoll(false);
		yawRateSignal = imu.getAngularVelocityZWorld(false);

		// Set update frequencies
		yawSignal.setUpdateFrequency(50);
		pitchSignal.setUpdateFrequency(50);
		rollSignal.setUpdateFrequency(50);
		yawRateSignal.setUpdateFrequency(50);

		// Optimize bus utilization — disable signals we don't use
		imu.optimizeBusUtilization();
	}

	public void sense(IMUInputs inputs) {
		if (!Robot.isReal()) return;

		BaseStatusSignal.waitForAll(0, yawSignal, pitchSignal, rollSignal, yawRateSignal);

		inputs.isConnected = imu.isConnected();
		inputs.isCalibrating = false;
		inputs.yawRadians = yawSignal.getValueAsDouble() / 180 * PI;
		inputs.pitchRadians = pitchSignal.getValueAsDouble() / 180 * PI;
		inputs.rollRadians = rollSignal.getValueAsDouble() / 180 * PI;
		inputs.yawTsSec = yawSignal.getTimestamp().getTime();
	}
}
