package frc.robot.Subsystems.Amp;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.MotionMagicConfigs;


public class AmpIO implements Amp{
    private TalonFX motor;
    private final TalonFXConfiguration configs;
    private final DutyCycleOut request;
    private final Slot0Configs slot0;
    private final MotionMagicConfigs motionMagicConfigs;
    // private double kP = constants.AMPConstants.kP; 
    // private double kI = constants.AMPConstants.kI;
    // private double kD = constants.AMPConstants.kD;

    
    public AmpIO(){
        
        motor = new TalonFX(Constants.AMPConstants);
        configs = new TalonFXConfiguration();
        Slot0 = configs.slot0();
        Slot0.kS = Constants.AMPConstants.kS;
        Slot0.kV = Constants.AMPConstants.kV;
        Slot0.kA = Constants.AMPConstants.kA;
        Slot0.kP = Constants.AMPConstants.kP;
        Slot0.kI = Constants.AMPConstants.kI;
        Slot0.kD = Constants.AMPConstants.kD;

        configs.MotorOutput.Inverted= InvertedValue.Clockwise_Positive;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        request = new DutyCycleOut(0).withEnableFOC(false);
        motionMagicConfigs = configs.MotionMagic;
        motionMagicConfigs.MotionmagicCruiseVelocity = Constants.AMPConstants.motionmagicCruiseVelocity;
        motionMagicConfigs.MotionmagicCruiseAcceleration = Constants.AMPConstants.MotionmagicCruiseAcceleration;
        motionMagicConfigs.MotionmagicCruiseJerk = Constants.AMPConstants.motionmagicCruiseJerk;

        motor.getConfigurator().apply(configs);



        

    }
}
