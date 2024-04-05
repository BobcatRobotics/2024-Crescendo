package frc.robot.Subsystems.CANdle;

import org.littletonrobotics.junction.AutoLog;


public interface CANdleIO{
    @AutoLog
    public static class CANdleIOInputs {
        public CANdleState state = CANdleState.OFF;
    }

    public default void setLEDs(CANdleState state){}
    public default void updateInputs(CANdleIOInputs inputs){}
    

}
