package frc.robot.subsystem.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotContainer.*;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class ShooterFlywheelSysID {
	public SysIdRoutine routine;

	public MutVoltage voltage = Volts.mutable(0);
	public MutDistance position = Meters.mutable(0);
	public MutLinearVelocity velocity = MetersPerSecond.mutable(0);

	public void actuate(Voltage volts) {
		shooter.hw.flywheelTalonFX.setVoltage(volts.magnitude());
	}

	public void log(SysIdRoutineLog log) {
		log.motor("flywheel")
				.voltage(voltage.mut_replace(shooter.inputs.flywheelVoltageVolts, Volts))
				.linearPosition(position.mut_replace(
						shooter.hw.flywheelTalonFX.getPosition().getValueAsDouble() * 2 * Math.PI, Meters))
				.linearVelocity(velocity.mut_replace(
						shooter.hw.flywheelTalonFX.getVelocity().getValueAsDouble() * 2 * Math.PI, MetersPerSecond));
	}

	public void init() {
		shooter.hw.sysIdFlywheel = true;

		SysIdRoutine.Config config = new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(10), Seconds.of(30));
		SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(this::actuate, this::log, shooter);
		routine = new SysIdRoutine(config, mechanism);

		controls.sysidQuasiForward().whileTrue(routine.quasistatic(Direction.kForward));
		controls.sysidQuasiBackward().whileTrue(routine.quasistatic(Direction.kReverse));
		controls.sysidDynaForward().whileTrue(routine.dynamic(Direction.kForward));
		controls.sysidDynaBackward().whileTrue(routine.dynamic(Direction.kReverse));
	}
}
