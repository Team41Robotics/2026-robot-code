package frc.robot.subsystem.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
	public ClimberHW hw = new ClimberHW();
	public ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

	public double targetVoltage = 0;
	public double actuatorTargetVoltage = 0;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("Climber", inputs);
	}

	public void actuate() {
		hw.actuate(targetVoltage, actuatorTargetVoltage);
	}
}
