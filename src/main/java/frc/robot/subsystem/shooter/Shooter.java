package frc.robot.subsystem.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
	public ShooterHW hw = new ShooterHW();
	public ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

	public double flyWheelVelocityRPM;
	public double hoodAngleRad;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Shooter", inputs);

		flyWheelVelocityRPM = inputs.flyWheelVelocityRPM;
		hoodAngleRad = inputs.hoodAngleRad;
	}

	public void setFlyWheelVelocity(double rpm) {
		hw.setFlyWheelVelocity(rpm);
	}

	public void setFlyWheelVoltage(double voltage) {
		hw.setFlyWheelVoltage(voltage);
	}

	public void stopFlyWheel() {
		hw.setFlyWheelVelocity(0);
	}

	public void setHoodAngle(double angleRad) {
		hw.setHoodAngle(angleRad);
	}

	public void setIndexerVoltage(double voltage) {
		hw.setIndexerVoltage(voltage);
	}
}
