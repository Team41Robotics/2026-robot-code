package frc.robot.subsystem.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class DriveSysID {
	SysIdRoutine routine;

	MutVoltage voltage = Volts.mutable(0);
	MutDistance distance = Meters.mutable(0);
	MutLinearVelocity velocity = MetersPerSecond.mutable(0);

	public void actuate(Voltage volts) {
		for (int i = 0; i < drive.modules.length; i++) {
			drive.modules[i].hw.driveTalonFX.setVoltage(volts.magnitude());
			drive.modules[i].drive(new SwerveModuleState(0, new Rotation2d()));
		}
	}

	public void log(SysIdRoutineLog log) {
		for (int i = 0; i < drive.modules.length; i++) {
			SwerveModule module = drive.modules[i];
			SwerveInputsAutoLogged inputs = module.inputs;
			log.motor(module.name)
					.voltage(voltage.mut_replace(inputs.driveVoltage, Volts))
					.linearPosition(distance.mut_replace(inputs.drivePos, Meters))
					.linearVelocity(velocity.mut_replace(inputs.driveVel, MetersPerSecond));
		}
	}

	public void init() {
		for (int i = 0; i < drive.modules.length; i++) {
			drive.modules[i].hw.sysidDrive = true;
		}

		SysIdRoutine.Config config = new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(5), Seconds.of(10));
		SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(this::actuate, this::log, drive);
		routine = new SysIdRoutine(config, mechanism);

		ctrl.sysid_quasi_forward().whileTrue(routine.quasistatic(Direction.kForward));
		ctrl.sysid_quasi_backward().whileTrue(routine.quasistatic(Direction.kReverse));
		ctrl.sysid_dyna_forward().whileTrue(routine.dynamic(Direction.kForward));
		ctrl.sysid_dyna_backward().whileTrue(routine.dynamic(Direction.kReverse));
	}
}
