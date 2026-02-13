package frc.robot.subsystem.drive;

public class SwerveModuleConfiguration {
	public final String name;
	public final int turnMotorId;
	public final int driveMotorId;
	public final int encoderId;
	public final double angleOffset;

	public SwerveModuleConfiguration(
			String name, int turnMotorId, int driveMotorId, int encoderId, double angleOffset) {
		this.name = name;
		this.turnMotorId = turnMotorId;
		this.driveMotorId = driveMotorId;
		this.encoderId = encoderId;
		this.angleOffset = angleOffset;
	}
}
