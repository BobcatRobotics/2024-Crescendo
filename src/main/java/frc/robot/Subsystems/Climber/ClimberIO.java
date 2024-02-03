package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

import java.lang.Object;
import edu.wpi.first.wpilibj.MotorSafety;
import com.ctre.phoenix6.wpiutils.MotorSafetyImplem;



public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs{

    }

    public default void updateClimberPosition(){

    }

    public default void deployClimber(){

    }

    public default void retractClimber(){
        
    }


}