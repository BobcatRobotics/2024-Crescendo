package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double switchMotorCurrent = 0.0;

        public double floorMotorCurrent = 0.0;

        public double outsideMotorCurrent = 0.0;

        public double tofValue = 0.0;

        public boolean intakeSensorTripped = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void switchMotorSetPercentOut(double percent) {}

    public default void switchMotorStop() {}

    public default void floorMotorSetPercentOut(double percent) {}

    public default void floorMotorStop() {}

    public default void outsideMotorSetPercentOut(double percent) {}

    public default double getFloorMotorCurrent() {return 0.0;}

    public default void outsideMotorStop() {}

    public default double getSwitchMotorCurrent() {return 0.0;}

    public default double getOuterMotorCurrent() {return 0.0;}
}
