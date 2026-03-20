package frc.robot.subsystem.indexer;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IndexerInputs {
	public double spinVelocityRPM;
	public double spinVoltageVolts;
	public double spinCurrentAmps;
	public double spinBusVoltageVolts;
	public double spinBusCurrentAmps;
	public double spinTsSec;

	public double elevatorVelocityRPM;
	public double elevatorVoltageVolts;
	public double elevatorCurrentAmps;
	public double elevatorBusVoltageVolts;
	public double elevatorBusCurrentAmps;
	public double elevatorTsSec;

	public double backvatorVelocityRPM;
	public double backvatorVoltageVolts;
	public double backvatorCurrentAmps;

	// Spark heartbeat (active transaction): "up" when a recent poll returned kOk
	public double backvatorLastGoodTimeSec;
	public double backvatorLastHeartbeatPollTimeSec;
	public int backvatorLastErrorCode;
}
