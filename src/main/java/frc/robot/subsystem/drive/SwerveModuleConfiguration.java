package frc.robot.subsystem.drive;

public class SwerveModuleConfiguration {
	public String name;

	public int turn_motor_id;
	public int drive_motor_id;
	public int encoder_id;

	public double angleOffset;

	public boolean turnInverted;

	public SwerveModuleConfiguration(
			String name,
			int turn_motor_id,
			int drive_motor_id,
			int encoder_id,
			double angleOffset,
			boolean turnInverted) {
		this.name = name;
		this.turn_motor_id = turn_motor_id;
		this.drive_motor_id = drive_motor_id;
		this.encoder_id = encoder_id;
		this.angleOffset = angleOffset;
		this.turnInverted = turnInverted;
	}
}
