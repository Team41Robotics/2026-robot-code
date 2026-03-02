package frc.robot.subsystem.leds;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.RainbowAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("static-access")
public class LEDS extends SubsystemBase {
	public static final RainbowAnimation DISABLED_ANIMATION = new RainbowAnimation(0, LEDSHW.TOTAL_LED_COUNT - 1);

	public LEDSHW hw = new LEDSHW();
	public LEDSInputsAutoLogged inputs = new LEDSInputsAutoLogged();

	public ControlRequest control = DISABLED_ANIMATION;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/LEDS", inputs);
	}

	public void actuate() {
		Logger.recordOutput("/LEDS/controlRequest", control.getName());

		hw.actuate(inputs, control);
	}
}
