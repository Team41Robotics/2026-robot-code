package frc.robot.subsystem.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotContainer.*;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class IntakeJointSysID {
	public SysIdRoutine routine;

	public MutVoltage voltage = Volts.mutable(0);
	public MutAngle position = Radians.mutable(0);
	public MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

	public void actuate(Voltage volts) {
		intake.hw.jointSparkMax.setVoltage(volts.magnitude());
	}

	public void log(SysIdRoutineLog log) {
		log.motor("joint")
				.voltage(voltage.mut_replace(intake.inputs.jointVoltageVolts, Volts))
				.angularPosition(position.mut_replace(intake.inputs.jointPosRadians, Radians))
				.angularVelocity(velocity.mut_replace(intake.inputs.jointVelRadiansPerSec, RadiansPerSecond));
	}

	public void init() {
		intake.hw.sysIdJoint = true;

		SysIdRoutine.Config config = new SysIdRoutine.Config(Volts.of(0.25).per(Second), Volts.of(3), Seconds.of(10));
		SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(this::actuate, this::log, intake);
		routine = new SysIdRoutine(config, mechanism);

		controls.sysidQuasiForward().whileTrue(routine.quasistatic(Direction.kForward));
		controls.sysidQuasiBackward().whileTrue(routine.quasistatic(Direction.kReverse));
		controls.sysidDynaForward().whileTrue(routine.dynamic(Direction.kForward));
		controls.sysidDynaBackward().whileTrue(routine.dynamic(Direction.kReverse));
	}
}
