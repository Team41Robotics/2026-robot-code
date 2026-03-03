package frc.robot.subsystem.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberHW {
	public TalonFX insideClimberTalon;
	public TalonFX outsideClimberTalon;

	public DigitalInput outsideLimitSwitchTop;
	public DigitalInput insideLimitSwitchTop;
	public DigitalInput outsideLimitSwitchBottom;
	public DigitalInput insideLimitSwitchBottom;

	public void init() {
		insideClimberTalon = new TalonFX(60);
		outsideClimberTalon = new TalonFX(61);

		outsideLimitSwitchTop = new DigitalInput(0); // TODO
		insideLimitSwitchTop = new DigitalInput(1); // TODO
		outsideLimitSwitchBottom = new DigitalInput(2); // TODO
		insideLimitSwitchBottom = new DigitalInput(3); // TODO

		insideClimberTalon.setNeutralMode(NeutralModeValue.Brake);
		outsideClimberTalon.setNeutralMode(NeutralModeValue.Brake);
	}

	public void sense(ClimberInputs inputs) {
		inputs.insideClimberPosMeters = insideClimberTalon.getPosition().getValueAsDouble();
		inputs.insideClimberVelMetersPerSec = insideClimberTalon.get();
		inputs.insideClimberVoltageVolts = insideClimberTalon.getMotorVoltage().getValueAsDouble();
		inputs.insideClimberCurrentAmps = insideClimberTalon.getStatorCurrent().getValueAsDouble();

		inputs.outsideClimberPosMeters = outsideClimberTalon.getPosition().getValueAsDouble();
		inputs.outsideClimberVelMetersPerSec = outsideClimberTalon.get();
		inputs.outsideClimberVoltageVolts =
				outsideClimberTalon.getMotorVoltage().getValueAsDouble();
		inputs.outsideClimberCurrentAmps =
				outsideClimberTalon.getStatorCurrent().getValueAsDouble();

		inputs.isOutsideLimitSwitchTopOn = outsideLimitSwitchTop.get();
		inputs.isInsideLimitSwitchTopOn = insideLimitSwitchTop.get();
		inputs.isOutsideLimitSwitchBottomOn = outsideLimitSwitchBottom.get();
		inputs.isInsideLimitSwitchBottomOn = insideLimitSwitchBottom.get();
	}

	public void actuate(double insideTargetVoltage, double outsideTargetVoltage) {
		insideClimberTalon.setVoltage(insideTargetVoltage);
		outsideClimberTalon.setVoltage(outsideTargetVoltage);
	}
}
