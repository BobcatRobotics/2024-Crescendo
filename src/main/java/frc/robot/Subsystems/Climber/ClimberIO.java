package frc.robot.Subsystems.Climber;
import org.littletonrobotics.junction.AutoLog;



public interface ClimberIO {
    // Start Advantage Kit Logging
    @AutoLog 
    public static class ClimberIOInputs {
        // These are the data points I want to collect about the climber
        public double climberMotorStatorCurrent = 0.0;
        public double climberMotorPosition = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setPercentOut(double percent) {}

    public default void holdPos(double rot) {}

    public default void stop() {}

    public default void climbMode(double pos) {}
}