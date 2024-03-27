package frc.robot.Subsystems.CANdle;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface CANdleIO{
    @AutoLog
    public static class CANdleIOInputs {
        public double[] rgb = {0,0,0};
        public CANdleState state = CANdleState.OFF;

    }

    public default void setLEDs(CANdleState state){}
    public default void turnOff(){}
    

}
