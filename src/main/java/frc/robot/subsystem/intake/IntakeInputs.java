package frc.robot.subsystem.intake;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IntakeInputs {
	public double jointPosRadians;
	public double jointVelRadiansPerSec;
	public double jointVoltageVolts;
	public double jointCurrentAmps;
	public double jointBusVoltageVolts;
	public double jointAbsTsSec;

	public double intakeVelocityRPM;
	public double intakeVoltageVolts;
	public double intakeCurrentAmps;
	public double intakeBusVoltageVolts;

	// Spark heartbeat (active transaction): "up" when a recent poll returned kOk
	public double jointLastGoodTimeSec;
	public double jointLastHeartbeatPollTimeSec;
	public int jointLastErrorCode;
	public double intakeLastGoodTimeSec;
	public double intakeLastHeartbeatPollTimeSec;
	public int intakeLastErrorCode;
}
