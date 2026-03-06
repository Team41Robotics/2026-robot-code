package frc.robot.commands.intake;

import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeUp extends Command {
	public static final double JOINT_UP_VOLTAGE = 3.0;
	public static final double JOINT_UP_POS = 110.0 / 180 * PI;
	public static final double TURRET_UP_POS = -PI / 2;

	public IntakeUp() {
		addRequirements(intake, shooter);
	}

	@Override
	public void execute() {
		intake.targetJointVoltage = intake.inputs.jointPosRadians < JOINT_UP_POS ? JOINT_UP_VOLTAGE : 0;
		intake.targetIntakeVoltage = 0;
		shooter.targetTurretPos = TURRET_UP_POS;
		shooter.targetFlywheelRPM = 0;
		shooter.targetHoodPos = 0;
	}

	@Override
	public void end(boolean interrupted) {
		intake.targetJointVoltage = 0;
		intake.targetIntakeVoltage = 0;
	}
}
