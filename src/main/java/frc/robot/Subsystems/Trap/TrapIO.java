package frc.robot.Subsystems.Trap;
import org.littletonrobotics.junction.AutoLog;


public interface TrapIO {
    // Start Advantage Kit Logging
    @AutoLog 
    public static class TrapIOInputs{
        public double trapPosition = 0.0;

        public double WinchMotorPercentOut = 0.0;
        public double WinchMotorStatorCurrent = 0.0;
        public double WinchMotorVelocityRPS = 0.0;
        public double WinchMotorPosition = 0.0;

        public double motionmagicAcceleration = 0.0;
        public double motionmagicCruiseVelocity = 0.0;
        public double motionmagicJerk = 0.0;

        public double ShooterMotorPercentOut = 0.0;
        public double ShooterMotorStatorCurrent = 0.0;
        public double ShooterMotorVelocityRPS = 0.0;
        public double ShooterMotorPosition = 0.0;
    }

}
