package frc.robot.subsystem.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
	public ClimberHW hw = new ClimberHW();
	public ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

	double insideTargetVoltage;
	double outsideTargetVoltage;

	public void init() {
		insideTargetVoltage = -5; // TODO
		outsideTargetVoltage = 5; // TODO
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
	}

	public void actuate() {
		if (inputs.isInsideLimitSwitchTopOn || inputs.isInsideLimitSwitchBottomOn) {
			insideTargetVoltage = -insideTargetVoltage;
		}
		if (inputs.isOutsideLimitSwitchTopOn || inputs.isOutsideLimitSwitchBottomOn) {
			outsideTargetVoltage = -outsideTargetVoltage;
		}
		hw.actuate(insideTargetVoltage, outsideTargetVoltage);
	}
}
