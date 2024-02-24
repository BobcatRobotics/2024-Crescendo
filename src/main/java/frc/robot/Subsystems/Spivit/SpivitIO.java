package frc.robot.Subsystems.Spivit;

import org.littletonrobotics.junction.AutoLog;

public interface SpivitIO{

    @AutoLog
    public static class SpivitIOInputs {
        public double angleMotorPosition = 0.0; //position in degrees for the shooter from the angle motor 
        public double angleMotorStatorCurrent = 0.0; //Current to the angle shooter motor
    }

    public default void updateInputs(SpivitIOInputs inputs) {}

    public default void setAngle(double angle){}
    public default double getAngle(){return 0.0;}
    public default void stopMotor(){}
    public default void stopMotorFeedforward(){}
    public default void setPercent(double percent){}
    public default boolean aligned(){return false;}

}