package frc.robot.subsystem.intake;

import static frc.robot.RobotContainer.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class IntakeHW {
	public TalonFX spinnerTalonFX;
	public VoltageOut controlRequest = new VoltageOut(0);

	public void init() {
		if (!Robot.isReal()) return;

		spinnerTalonFX = new TalonFX(22, driveBus);
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 40; // FIXME. supply current limit (A)
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = 60; // FIXME. stator current limit (A)

		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // FIXME. inversion

		spinnerTalonFX.getConfigurator().apply(config);
		spinnerTalonFX.clearStickyFaults();
		spinnerTalonFX.setNeutralMode(NeutralModeValue.Coast);
	}

	public void sense(IntakeInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.voltage = spinnerTalonFX.getMotorVoltage().getValueAsDouble();
		inputs.current = spinnerTalonFX.getStatorCurrent().getValueAsDouble();
		inputs.busVoltage = spinnerTalonFX.getSupplyVoltage().getValueAsDouble();
		inputs.busCurrent = spinnerTalonFX.getSupplyCurrent().getValueAsDouble();
		inputs.vel = spinnerTalonFX.getVelocity().getValueAsDouble();
	}

	public void actuate(double voltage) {
		Logger.recordOutput("/Intake/actuatedVoltage", voltage);

		if (!Robot.isReal()) return;

		spinnerTalonFX.setControl(controlRequest.withOutput(voltage));
	}
}
