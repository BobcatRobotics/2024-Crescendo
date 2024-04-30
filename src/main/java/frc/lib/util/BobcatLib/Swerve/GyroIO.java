package frc.lib.util.BobcatLib.Swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;

        public Rotation2d yawPosition = new Rotation2d();
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
        public double zAngularVelocityDegPerSec = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    /**
     * Sets the current Gyro yaw
     * @param yaw value to set yaw to, in degrees
     */
    public default void setYaw(double yaw) {}
}
