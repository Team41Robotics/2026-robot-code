package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterStartup extends Command {
	public ShooterStartup() {
		addRequirements(shooter);
	}

	public boolean turretLimitSwitchTriggered = false;
	public boolean hoodLimitSwitchTriggered = false;

	@Override
	public void execute() {
		shooter.targetFlywheelRPM = 0;
		shooter.targetTurretPos = 20 * PI / 180.0;
		shooter.targetHoodPos = 0;
		// if (shooter.inputs.turretLimitSwitchOn) {
		turretLimitSwitchTriggered = true;
		// }
		hoodLimitSwitchTriggered = true; // hood limit switch is dead
	}

	@Override
	public boolean isFinished() {
		return turretLimitSwitchTriggered && hoodLimitSwitchTriggered;
	}

	@Override
	public void end(boolean interrupted) {
		if (!interrupted) {
			shooter.zeroed = true;
		}
		shooter.targetFlywheelRPM = 0;
		shooter.targetTurretPos = shooter.inputs.turretPosRadians;
		shooter.targetHoodPos = shooter.inputs.hoodPosRadians;
	}
}
