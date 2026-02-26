package frc.robot.commands.shooter;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

public class SetShooterPosition extends Command {
	public double turretPos;
	public double hoodPos;

	public SetShooterPosition(double turretPos, double hoodPos) {
		this.turretPos = turretPos;
		this.hoodPos = hoodPos;
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		shooter.targetTurretPos = turretPos;
		shooter.targetHoodPos = hoodPos;
	}
}
