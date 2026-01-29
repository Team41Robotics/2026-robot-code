package frc.robot.subsystem.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.left_js;

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
			SwerveHW hw = module.hw;
			log.motor(module.name)
					.voltage(voltage.mut_replace(hw.turnVoltage, Volts))
					.linearPosition(distance.mut_replace(hw.turnPos, Meters))
					.linearVelocity(velocity.mut_replace(hw.turnVel, MetersPerSecond));
		}
	}

	public void init() {
		SysIdRoutine.Config config = new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(5), Seconds.of(15));
		SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(this::actuate, this::log, drive);
		routine = new SysIdRoutine(config, mechanism);

		left_js.button(1).whileTrue(routine.quasistatic(Direction.kForward));
		left_js.button(2).whileTrue(routine.quasistatic(Direction.kReverse));
		left_js.button(3).whileTrue(routine.dynamic(Direction.kForward));
		left_js.button(4).whileTrue(routine.dynamic(Direction.kReverse));
	}
}
