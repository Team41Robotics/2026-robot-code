package frc.robot.subsystem.shooter;


public class Shooter  {

        public ShooterHW hw;
        public ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

        public Shooter() {
                hw = new ShooterHW();
        }
        public void init() {
                sense();
        }
        public void sense() {
                hw.sense(inputs);
        }       

       
        public void stopFlyWheel() {
                hw.setFlyWheelVelocity(0);
        }
}       