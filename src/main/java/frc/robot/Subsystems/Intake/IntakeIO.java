package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double outerMotorPercentOut = 0.0;
        public double outerMotorCurrent = 0.0;
        public double outerMotorVelocityRotPerSec = 0.0;

        public double middleMotorPercentOut = 0.0;
        public double middleMotorCurrent = 0.0;
        public double middleMotorVelocityRotPerSec = 0.0;

        public double innerMotorPercentOut = 0.0;
        public double innerMotorCurrent = 0.0;
        public double innerMotorVelocityRotPerSec = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void outerMotorSetPercentOut(double percent) {}

    public default void outerMotorStop() {}

    public default void middleMotorSetPercentOut(double percent) {}

    public default void middleMotorStop() {}

    public default void innerMotorSetPercentOut(double percent) {}

    public default void innerMotorStop() {}
}
