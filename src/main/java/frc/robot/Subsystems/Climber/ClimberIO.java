package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

import java.lang.Object;
import edu.wpi.first.wpilibj.MotorSafety;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.wpiutils.MotorSafetyImplem;


public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs{
        public double climberPosition = 0.0;
        ClimberIOFalcon climberMotor = new ClimberIOFalcon(Constants.climberConstants.MotorID);
        public double climberMotorPercentOut = 0.0;
        public double climberMotorStatorCurrent = 0.0;
        public double climberMotorVelocityRPS = 0.0;
        public double climberMotorPosition = 0.0;
    }


    public default double getClimberPosition(int climberEncoderValue){
        return 0.0;
    }

    public default void updateConfigs(){

    }


    /* 
    public default void updateClimberPosition(){

    }

    public default void deployClimber(){

    }

    public default void retractClimber(){

    }

    public default void updateConfigs(){

    }

*/

}