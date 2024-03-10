package frc.robot.Subsystems.Amp;

import org.littletonrobotics.junction.AutoLog;

public interface AmpIO {
    @AutoLog
    public static class AmpIOInputs {
        public double motorPosition = 0.0; //in degrees
        public double current = 0;
    }

    public default void setPos(double rotationAmount) {}
    public default void setPercent(double percent){}
    public default void stop() {}
    public default void updateInputs(AmpIOInputs inputs) {}
    public default void zeroPosition() {}
    public default boolean currentLimitReached(){return false;}
    public default void stopMotorFeedforward(){}


}
