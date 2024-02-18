package frc.robot.Subsystems.Trap;
import org.littletonrobotics.junction.AutoLog;


public interface TrapIO {
    // Start Advantage Kit Logging
    @AutoLog 
    public static class TrapIOInputs{
        public double trapPosition = 0.0;
    }

    public default void updateInputs(TrapIOInputs inputs) {}

    public default void setPosition(double deg) {}
    public default void setArmPercent(double percent) {}
    public default void stopArm() {}

    public default void setRollerPercentOut(double percent) {}
    public default void stopRollers() {}
}
