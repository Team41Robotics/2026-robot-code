package frc.robot.subsystem.shooter;

import static frc.robot.RobotContainer.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.shooter.ShooterStartup;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
	public ShooterHW hw = new ShooterHW();
	public ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

	public double targetFlywheelRPM = 0;
	public boolean onTarget = false;

	public void init() {
		hw.init();
		sense();
	}

	public void sense() {
		hw.sense(inputs);
		Logger.processInputs("/Shooter", inputs);

		if (robot.isTeleopEnabled()) {
			CommandScheduler.getInstance().schedule(new ShooterStartup());
		}
	}

	public void actuate() {
		Logger.recordOutput("/Shooter/targetFlywheelRPM", targetFlywheelRPM);
		
		hw.actuate(inputs, targetFlywheelRPM);
	}
}
