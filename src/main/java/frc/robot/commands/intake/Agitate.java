package frc.robot.commands.intake;

import static frc.robot.RobotContainer.intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
public class Agitate extends Command {
        public static final double INTAKE_VOLTAGE = -12.0; // TUNEME

        public static final TrapezoidProfile profile = new TrapezoidProfile(new Constraints(1, 0.5));// TUNEME. max velocity, max acceleration);;-
        public Agitate() {
                addRequirements(intake);
        }
        @Override
        public void initialize() {
        }
        @Override
        public void execute() {
                intake.targetIntakeVoltage = INTAKE_VOLTAGE;
        }

        @Override
        public void end(boolean interrupted) {
                intake.targetIntakeVoltage = 0;
        }        
}