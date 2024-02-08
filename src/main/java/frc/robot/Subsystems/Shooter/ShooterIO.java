package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double topMotorStatorCurrent = 0.0;
        public double topMotorVelocityRPS = 0.0;

        public double bottomMotorStatorCurrent = 0.0;
        public double bottomMotorVelocityRPS = 0.0;

        public double angleMotorPosition = 0.0;
        public double angleMotorStatorCurrent = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setTopVelocity(double rps) {}
    public default void setBottomVelocity(double rps) {}
    public default void setAngle(double deg) {}

    public default void stopTopMotor() {}
    public default void stopBottomMotor() {}
    public default void stopAngleMotor() {}
}