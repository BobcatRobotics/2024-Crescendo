package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double topMotorStatorCurrent = 0.0; //Current to the top shooter motor
        public double topMotorVelocityRPS = 0.0; //Revs per sec for top motor

        public double bottomMotorStatorCurrent = 0.0; //Current to the bttom shooter motor
        public double bottomMotorVelocityRPS = 0.0; //Revs per sec for bottom motor
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setTopVelocity(double rps) {} //revs per sec
    public default void setBottomVelocity(double rps) {} //revs per sec
    public default double getTopVelocity() {return 0.0;} //rps
    public default double getBottomVelocity() {return 0.0;} //rps
    public default void stopTopMotor() {} //STOP IT. STOP THE TOP MOTOR :(( 
    public default void stopBottomMotor() {} //STOP THE BOTTOM MOTOR!!!

}