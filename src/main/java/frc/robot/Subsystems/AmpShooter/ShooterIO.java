package frc.robot.Subsystems.AmpShooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double percentOut = 0.0;
        public double statorCurrent = 0.0;
        public double velocityRPS = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void updateConfigs() {}

    public default void setVelocity(double percent) {}

    public default void Stopmotor(){}

    public default double[] get_rpm(){return null;}
}
