package frc.robot.subsystem.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
	public enum Mode {
		VELOCITY,
		VOLTAGE
	}

	public ShooterHW hw = new ShooterHW();
	public ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

	public double shooterVelocityRPM;

	public Mode mode = Mode.VELOCITY;
	public double targetRPM = 0;
	public double targetVoltage = 0;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Shooter", inputs);

		shooterVelocityRPM = inputs.velocityRPM;
	}

	public void actuate() {
		if (mode == Mode.VELOCITY) {
			hw.setShooterVelocity(targetRPM);
		} else {
			hw.setShooterVoltage(targetVoltage);
		}
		Logger.recordOutput("/Shooter/mode", mode.name());
		Logger.recordOutput("/Shooter/targetRPM", targetRPM);
		Logger.recordOutput("/Shooter/targetVoltage", targetVoltage);
	}

	public void setVelocity(double rpm) {
		mode = Mode.VELOCITY;
		targetRPM = rpm;
	}

	public void setVoltage(double voltage) {
		mode = Mode.VOLTAGE;
		targetVoltage = voltage;
	}

	public void stop() {
		mode = Mode.VOLTAGE;
		targetVoltage = 0;
	}
}
