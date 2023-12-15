package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public double yawPositionDeg = 0.0;
        public double pitchPositionDeg = 0.0;
        public double rollPositionDeg = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void setYaw(double yaw) {}
}
