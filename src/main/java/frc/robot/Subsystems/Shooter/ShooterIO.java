package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double topMotorPercentOut = 0.0;
        public double topMotorStatorCurrent = 0.0;
        public double topMotorVelocityRPS = 0.0;

        public double bottomMotorPercentOut = 0.0;
        public double bottomMotorStatorCurrent = 0.0;
        public double bottomMotorVelocityRPS = 0.0;

        public double angleMotorPercentOut = 0.0;
        public double angleMotorStatorCurrent = 0.0;
        public double angleMotorVelocityRPS = 0.0;

        public double feederMotorPercentOut = 0.0;
        public double feederMotorStatorCurrent = 0.0;
        public double feederMotorVelocityRPS = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void updateConfigs() {}

    public default void bottomMotorSetPercentOut(double percent) {}
    public default void topMotorSetPercentOut(double percent) {}
    public default void feederMotorSetPercentOut(double percent) {}
    public default void angleMotorSetPercentOut(double percent) {}

    public default void bottomMotorStop() {}
    public default void topMotorStop() {}
    public default void angleMotorStop() {}
    public default void feederMotorStop() {}
}