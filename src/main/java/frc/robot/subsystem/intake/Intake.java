package frc.robot.subsystem.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
	public IntakeHW hw = new IntakeHW();
	public IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

	public double targetExtensionPosition = 0;
	public double targetIntakeVoltage = 0;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Intake", inputs);
	}

	public void actuate() {
		Logger.recordOutput("/Intake/extensionTargetPositionMeters", targetExtensionPosition);

		hw.actuate(inputs, targetExtensionPosition, targetIntakeVoltage);
	}
}
