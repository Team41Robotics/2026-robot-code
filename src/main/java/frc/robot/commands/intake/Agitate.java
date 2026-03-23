package frc.robot.commands.intake;

import static frc.robot.RobotContainer.intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
public class Agitate extends Command {
        public static final double INTAKE_VOLTAGE = -12.0; // TUNEME
        public static final double EXTENSION_IN_POSITION = 0.0; // TUNEME. retracted position (m)

        public static final TrapezoidProfile profile = new TrapezoidProfile(new Constraints(1, 0.5));// TUNEME. max velocity, max acceleration

        public State setpoint = new State();

        public Agitate() {
                addRequirements(intake);
        }
        @Override
        public void initialize() {
                setpoint = new State(intake.inputs.extensionPosMeters, 0);
        }
        @Override
        public void execute() {
                intake.targetIntakeVoltage = INTAKE_VOLTAGE;
                setpoint = profile.calculate(0.02, setpoint, new State(EXTENSION_IN_POSITION, 0));
                intake.targetExtensionPosition = setpoint.position;
        }

        @Override
        public void end(boolean interrupted) {
                intake.targetIntakeVoltage = 0;
                intake.targetExtensionPosition = intake.inputs.extensionPosMeters;
        }
}