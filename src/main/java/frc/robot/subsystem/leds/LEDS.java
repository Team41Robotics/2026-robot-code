package frc.robot.subsystem.leds;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("static-access")
public class LEDS extends SubsystemBase {
	public static final RainbowAnimation DISABLED_ANIMATION = new RainbowAnimation(0, LEDSHW.TOTAL_LED_COUNT - 1);
	public static final SolidColor SHOOTING_ANIMATION =
			new SolidColor(0, LEDSHW.TOTAL_LED_COUNT - 1).withColor(new RGBWColor(Color.kBlue));
	public static final SolidColor IDLE_ANIMATION =
			new SolidColor(0, LEDSHW.TOTAL_LED_COUNT - 1).withColor(new RGBWColor(Color.kRed));

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
		if (control instanceof SolidColor) hw.actuate(inputs, new EmptyAnimation(0));
	}
}
