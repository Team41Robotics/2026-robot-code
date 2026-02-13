package frc.robot.subsystem.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotContainer.*;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class TurnSysID {
	SysIdRoutine routine;

	MutVoltage voltage = Volts.mutable(0);
	MutDistance distance = Meters.mutable(0);
	MutLinearVelocity velocity = MetersPerSecond.mutable(0);

	public void actuate(Voltage volts) {
		for (int i = 0; i < drive.modules.length; i++) {
			drive.modules[i].hw.turnTalonFX.setVoltage(volts.magnitude());
		}
	}

	public void log(SysIdRoutineLog log) {
		for (int i = 0; i < drive.modules.length; i++) {
			SwerveModule module = drive.modules[i];
			SwerveInputsAutoLogged inputs = module.inputs;
			log.motor(module.name)
					.voltage(voltage.mut_replace(inputs.turnVoltage, Volts))
					.linearPosition(distance.mut_replace(inputs.turnPos, Meters))
					.linearVelocity(velocity.mut_replace(inputs.turnVel, MetersPerSecond));
		}
	}

	public void init() {
		for (int i = 0; i < drive.modules.length; i++) {
			drive.modules[i].hw.sysidTurn = true;
		}

		SysIdRoutine.Config config = new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(5), Seconds.of(15)); // FIXME. sysid volt ramp, max volts, duration
		SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(this::actuate, this::log, drive);
		routine = new SysIdRoutine(config, mechanism);

		ctrl.sysid_quasi_forward().whileTrue(routine.quasistatic(Direction.kForward));
		ctrl.sysid_quasi_backward().whileTrue(routine.quasistatic(Direction.kReverse));
		ctrl.sysid_dyna_forward().whileTrue(routine.dynamic(Direction.kForward));
		ctrl.sysid_dyna_backward().whileTrue(routine.dynamic(Direction.kReverse));
	}
}
