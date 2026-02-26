package frc.robot.subsystem.intake;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IntakeInputs {
	public double jointPosRadians;
	public double jointVelRadiansPerSec;
	public double jointVoltageVolts;
	public double jointCurrentAmps;
	public double jointBusVoltageVolts;

	public double intakeVelocityRPM;
	public double intakeVoltageVolts;
	public double intakeCurrentAmps;
	public double intakeBusVoltageVolts;
	public double intakeBusCurrentAmps;
}
