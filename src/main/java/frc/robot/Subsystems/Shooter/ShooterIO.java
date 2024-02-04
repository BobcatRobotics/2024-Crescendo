package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double topMotorVelocityOut = 0.0;
        public double topMotorStatorCurrent = 0.0;
        public double topMotorVelocityRPS = 0.0;

        public double bottomMotorVelocityOut = 0.0;
        public double bottomMotorStatorCurrent = 0.0;
        public double bottomMotorVelocityRPS = 0.0;

        public double angleMotorPosition = 0.0;
        public double angleMotorStatorCurrent = 0.0;
        public double angleMotorVelocityRPS = 0.0;

        public double feederMotorPercentOut = 0.0;
        public double feederMotorStatorCurrent = 0.0;
        public double feederMotorVelocityRPS = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void updateConfigs() {}

    public default void bottomMotorSetVelocityOut(double velocity) {}
    public default void topMotorSetVelocityOut(double velocity) {}
    public default void feederMotorSetPercentOut(double percent) {}
    public default void angleMotorSetPosition(double position) {}

    public default void bottomMotorStop() {}
    public default void topMotorStop() {}
    public default void angleMotorStop() {}
    public default void feederMotorStop() {}
}