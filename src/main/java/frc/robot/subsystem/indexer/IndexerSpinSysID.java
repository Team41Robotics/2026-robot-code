package frc.robot.subsystem.indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotContainer.*;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class IndexerSpinSysID {
	public SysIdRoutine routine;

	public MutVoltage voltage = Volts.mutable(0);
	public MutAngle position = Radians.mutable(0);
	public MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

	public void actuate(Voltage volts) {
		indexer.hw.spinTalonFX.setVoltage(volts.magnitude());
	}

	public void log(SysIdRoutineLog log) {
		log.motor("spin")
				.voltage(voltage.mut_replace(indexer.inputs.spinVoltageVolts, Volts))
				.angularPosition(position.mut_replace(
						indexer.hw.spinTalonFX.getPosition().getValueAsDouble() * 2 * Math.PI, Radians))
				.angularVelocity(
						velocity.mut_replace(indexer.inputs.spinVelocityRPM * Math.PI / 30.0, RadiansPerSecond));
	}

	public void init() {
		indexer.hw.sysIdSpin = true;

		SysIdRoutine.Config config =
				new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(6), Seconds.of(10)); // TUNEME
		SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(this::actuate, this::log, indexer);
		routine = new SysIdRoutine(config, mechanism);

		controls.sysidQuasiForward().whileTrue(routine.quasistatic(Direction.kForward));
		controls.sysidQuasiBackward().whileTrue(routine.quasistatic(Direction.kReverse));
		controls.sysidDynaForward().whileTrue(routine.dynamic(Direction.kForward));
		controls.sysidDynaBackward().whileTrue(routine.dynamic(Direction.kReverse));
	}
}
