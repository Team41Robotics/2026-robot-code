package frc.robot.subsystem.intake;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
	public IntakeHW hw = new IntakeHW();
	public IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

	public double targetJointVoltage = 0;
	public double targetIntakeVoltage = 0;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Intake", inputs);

		if (robot.isDisabled()) {
			targetJointVoltage = 0;
		}
	}

	public void actuate() {
		Logger.recordOutput("/Intake/intakeVoltageVolts", targetIntakeVoltage);
		Logger.recordOutput("/Intake/jointTargetVoltageVolts", targetJointVoltage);

		hw.actuate(inputs, targetJointVoltage, targetIntakeVoltage);
	}
}
