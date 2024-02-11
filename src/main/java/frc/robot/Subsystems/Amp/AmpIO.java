package frc.robot.Subsystems.Amp;

import org.littletonrobotics.junction.AutoLog;

public interface AmpIO {
    @AutoLog
    public static class AmpIOInputs {
        public double motorposition = 0.0;
    }

    public default void setPos(double rotationAmount) {}
    public default void stop() {}
    public default void updateInputs(AmpIOInputs inputs) {}


}
