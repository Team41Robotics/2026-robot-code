package frc.robot.subsystem.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotContainer.*;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class ShooterTurretSysID {
	public SysIdRoutine routine;

	public MutVoltage voltage = Volts.mutable(0);
	public MutAngle position = Radians.mutable(0);
	public MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

	public void actuate(Voltage volts) {
		shooter.hw.turretTalonFX.setVoltage(volts.magnitude());
	}

	public void log(SysIdRoutineLog log) {
		log.motor("turret")
				.voltage(voltage.mut_replace(shooter.inputs.turretVoltageVolts, Volts))
				.angularPosition(position.mut_replace(shooter.inputs.turretPosRadians, Radians))
				.angularVelocity(velocity.mut_replace(shooter.inputs.turretVelRadiansPerSec, RadiansPerSecond));
	}

	public void init() {
		shooter.hw.sysIdTurret = true;

		SysIdRoutine.Config config = new SysIdRoutine.Config(Volts.of(0.25).per(Second), Volts.of(3), Seconds.of(10));
		SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(this::actuate, this::log, shooter);
		routine = new SysIdRoutine(config, mechanism);

		controls.sysidQuasiForward().whileTrue(routine.quasistatic(Direction.kForward));
		controls.sysidQuasiBackward().whileTrue(routine.quasistatic(Direction.kReverse));
		controls.sysidDynaForward().whileTrue(routine.dynamic(Direction.kForward));
		controls.sysidDynaBackward().whileTrue(routine.dynamic(Direction.kReverse));
	}
}
