package frc.robot.subsystem.climber;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ClimberInputs {
	public double insideClimberPosMeters;
	public double insideClimberVelMetersPerSec;
	public double insideClimberVoltageVolts;
	public double insideClimberCurrentAmps;

	public double outsideClimberPosMeters;
	public double outsideClimberVelMetersPerSec;
	public double outsideClimberVoltageVolts;
	public double outsideClimberCurrentAmps;

	public boolean isOutsideLimitSwitchTopOn;
	public boolean isInsideLimitSwitchTopOn;
	public boolean isOutsideLimitSwitchBottomOn;
	public boolean isInsideLimitSwitchBottomOn;
}
