package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double topMotorStatorCurrent = 0.0; //Current to the top shooter motor
        public double topMotorVelocityRPS = 0.0; //Revs per sec for top motor

        public double bottomMotorStatorCurrent = 0.0; //Current to the bttom shooter motor
        public double bottomMotorVelocityRPS = 0.0; //Revs per sec for bottom motor

        public double angleMotorPosition = 0.0; //position in degrees for the shooter from the angle motor 
        public double angleMotorStatorCurrent = 0.0; //Current to the angle shooter motor
        public double angleMotorRequestedPos = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setTopVelocity(double rps) {} //revs per sec
    public default void setBottomVelocity(double rps) {} //revs per sec
    public default void setAngle(double deg) {} //degrees!!!!!
    public default void setVelocityTune(double rpm){}//tuning purposes only
    public default void stopTopMotor() {} //STOP IT. STOP THE TOP MOTOR :(( 
    public default void stopBottomMotor() {} //STOP THE BOTTOM MOTOR!!!
    public default void stopAngleMotor() {} //STOP THE ANGLE MOTOR STOP IT RIGHT NOW!!!
    public default void setPercentOut(double percent) {}
}