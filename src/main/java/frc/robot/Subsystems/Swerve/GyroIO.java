package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public double yawPositionDeg = 0.0;
        public double pitchPositionDeg = 0.0;
        public double rollPositionDeg = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    /**
     * Sets the current Gyro yaw
     * @param yaw value to set yaw to, in degrees
     */
    public default void setYaw(double yaw) {}
}
