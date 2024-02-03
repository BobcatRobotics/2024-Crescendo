package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

import java.lang.Object;
import edu.wpi.first.wpilibj.MotorSafety;
import com.ctre.phoenix6.wpiutils.MotorSafetyImplem;
import com.ctre.phoenix6.jni.CtreJniWrapper;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.hardware.TalonFX;



public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs{
        public double climberPosition = 0.0;
        public double climberEncoderValue = 0.0;
    }

    public default double getClimberPosition(int climberEncoderValue){
        return 0.0;
    }

    public default void updateClimberPosition(){

    }

    public default void deployClimber(){

    }

    public default void retractClimber(){

    }


}