package frc.robot.subsystem.leds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;

public class LEDSHW {
	public static final int ONBOARD_LED_COUNT = 8;
	public static final int STRIP_LENGTH = 40;
	public static final int TOTAL_LED_COUNT = ONBOARD_LED_COUNT + STRIP_LENGTH;

	public CANdle candle;

	public StatusSignal<Voltage> supplyVoltage;
	public StatusSignal<Current> outputCurrent;
	public StatusSignal<Voltage> fiveVRailVoltage;
	public StatusSignal<Temperature> deviceTemp;
	public StatusSignal<Double> vBatModulation;

	public void init() {
		if (!Robot.isReal()) return;

		candle = new CANdle(6);

		CANdleConfiguration config = new CANdleConfiguration();
		config.LED.BrightnessScalar = 1.0;
		config.LED.StripType = StripTypeValue.GRB;
		config.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
		config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Modulated;
		config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

		candle.getConfigurator().apply(config);
		candle.clearStickyFaults();

		supplyVoltage = candle.getSupplyVoltage(false);
		outputCurrent = candle.getOutputCurrent(false);
		fiveVRailVoltage = candle.getFiveVRailVoltage(false);
		deviceTemp = candle.getDeviceTemp(false);
		vBatModulation = candle.getVBatModulation(false);

		supplyVoltage.setUpdateFrequency(50);
		outputCurrent.setUpdateFrequency(50);
		fiveVRailVoltage.setUpdateFrequency(50);
		deviceTemp.setUpdateFrequency(50);
		vBatModulation.setUpdateFrequency(50);

		candle.optimizeBusUtilization();
	}

	public void sense(LEDSInputs inputs) {
		if (!Robot.isReal()) return;

		BaseStatusSignal.refreshAll(supplyVoltage, outputCurrent, fiveVRailVoltage, deviceTemp, vBatModulation);

		inputs.busVoltageVolts = supplyVoltage.getValueAsDouble();
		inputs.outputCurrentAmps = outputCurrent.getValueAsDouble();
		inputs.fiveVRailVoltageVolts = fiveVRailVoltage.getValueAsDouble();
		inputs.boardTempCelsius = deviceTemp.getValueAsDouble();
		inputs.vBatModulation = vBatModulation.getValueAsDouble();
		inputs.tsSec = supplyVoltage.getTimestamp().getTime();
	}

	public void actuate(LEDSInputs inputs, ControlRequest request) {
		if (!Robot.isReal()) return;
		candle.setControl(request);
	}
}
