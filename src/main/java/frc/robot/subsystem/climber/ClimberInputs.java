package frc.robot.subsystem.climber;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ClimberInputs {
	public double posMeters;
	public double velMetersPerSec;
	public double voltageVolts;
	public double currentAmps;

	public boolean limitTop;
	public boolean limitBottom;
}
