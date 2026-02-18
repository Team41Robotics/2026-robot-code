package frc.robot.subsystem.shooter;



import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class ShooterHW {

        public TalonFX flyWheelMotor;
        public TalonFX flyWheel2Motor;
        public TalonFX hoodMotor;
        public TalonFX indexer;

        public PIDController flywheelPid;        
        public PIDController hoodPid;

        public ShooterHW() {
                flyWheelMotor = new TalonFX(0);
                flyWheel2Motor = new TalonFX(2);
                hoodMotor = new TalonFX(1);
                indexer = new TalonFX(3);

                flyWheel2Motor.setControl(new Follower(0, MotorAlignmentValue.Opposed));

                flywheelPid = new PIDController(0, 0.0, 0.0);
                hoodPid = new PIDController(0.5, 0.1, 0.0);
        } 

         public void setFlyWheelVelocity(double velocity) {
                flyWheelMotor.set(flywheelPid.calculate(velocity));
                 
         }

         public void setFlyWheelVoltage(double voltage) {
                flyWheelMotor.setVoltage(voltage); 
         }

         public void setHoodAngle(double angleRad) {
                hoodMotor.set(MathUtil.clamp(hoodPid.calculate(angleRad), -1, 1));
         }
  
         public void sense(ShooterInputs inputs) {
                inputs.flyWheelVelocity = flyWheelMotor.get();
                inputs.flyWheelCurrent = new double[] {flyWheelMotor.getStatorCurrent().getValueAsDouble()};
                inputs.flyWheelVoltage = flyWheelMotor.getMotorVoltage().getValueAsDouble();


                inputs.hoodAngleRad = hoodMotor.getPosition().getValueAsDouble() * 2.0 * Math.PI;
         }
        
}
