package frc.robot.subsystem.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import frc.robot.Robot;

public class LEDSHW {
	public static final int ONBOARD_LED_COUNT = 8;
	public static final int STRIP_LENGTH = 60; // TUNEME
	public static final int TOTAL_LED_COUNT = ONBOARD_LED_COUNT + STRIP_LENGTH;

	public CANdle candle;

	public void init() {
		if (!Robot.isReal()) return;

		candle = new CANdle(1434); // HACK

		CANdleConfiguration config = new CANdleConfiguration();
		config.LED.BrightnessScalar = 1.0;
		config.LED.StripType = StripTypeValue.GRB; // TUNEME: match your strip wiring
		config.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
		config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Modulated;
		config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

		candle.getConfigurator().apply(config);
		candle.clearStickyFaults();
	}

	public void sense(LEDSInputs inputs) {
		if (!Robot.isReal()) return;

		inputs.busVoltageVolts = candle.getSupplyVoltage().getValueAsDouble();
		inputs.outputCurrentAmps = candle.getOutputCurrent().getValueAsDouble();
		inputs.fiveVRailVoltageVolts = candle.getFiveVRailVoltage().getValueAsDouble();
		inputs.boardTempCelsius = candle.getDeviceTemp().getValueAsDouble();
		inputs.vBatModulation = candle.getVBatModulation().getValueAsDouble();
	}

	public void actuate(LEDSInputs inputs, ControlRequest request) {
		if (!Robot.isReal()) return;
		candle.setControl(request);
	}
}
