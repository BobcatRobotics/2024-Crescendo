package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double switchMotorPercentOut = 0.0;
        public double switchMotorCurrent = 0.0;
        public double switchMotorVelocityRotPerSec = 0.0;

        public double floorMotorPercentOut = 0.0;
        public double floorMotorCurrent = 0.0;
        public double floorMotorVelocityRotPerSec = 0.0;

        public double outsideMotorPercentOut = 0.0;
        public double outsideMotorCurrent = 0.0;
        public double outsideMotorVelocityRotPerSec = 0.0;

        public boolean tofValue = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void switchMotorSetPercentOut(double percent) {}

    public default void switchMotorStop() {}

    public default void floorMotorSetPercentOut(double percent) {}

    public default void floorMotorStop() {}

    public default void outsideMotorSetPercentOut(double percent) {}

    public default void outsideMotorStop() {}
}
