package frc.robot.Subsystems.Trap;
import org.littletonrobotics.junction.AutoLog;


public interface TrapIO {
    // Start Advantage Kit Logging
    @AutoLog 
    public static class TrapIOInputs{

        // The following applies to the shoulder motor
        public double trapPosition = 0.0;
        public double WinchMotorPercentOut = 0.0;
        public double WinchMotorStatorCurrent = 0.0;
        public double WinchMotorVelocityRPS = 0.0;
        public double WinchMotorPosition = 0.0;

        public double motionmagicAcceleration = 0.0;
        public double motionmagicCruiseVelocity = 0.0;
        public double motionmagicJerk = 0.0;

        // The following applies to the shooter motor on the trap
        public double ShooterMotorPercentOut = 0.0;
        public double ShooterMotorStatorCurrent = 0.0;
        public double ShooterMotorVelocityRPS = 0.0;
    }

    // No logic functions

    public default void updateInputs(TrapIOInputs i){
    }

    public default void runShooterMotor(double rps){
    }

    public default void stopShooterMotor(){
    }

    public default void setVelocityTune(double rpm){
    }

    public default void extendTrap(double rotationAmount){
    }

    public default void inverseTrapDirection(){
    }

    public default void stopTrapMotion(){
    }

}
