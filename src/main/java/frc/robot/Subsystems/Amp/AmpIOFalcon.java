package frc.robot.Subsystems.Amp;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;


public class AmpIOFalcon implements AmpIO{
    private TalonFX motor;
    private final TalonFXConfiguration configs;
    private final DutyCycleOut request;
    private final Slot0Configs slot0;
    private final MotionMagicConfigs motionMagicConfigs;
    // private final MotionMagicVoltage m_request;
    private final MotionMagicVoltage m_request;
    // private double kP = constants.AMPConstants.kP; 
    // private double kI = constants.AMPConstants.kI;
    // private double kD = constants.AMPConstants.kD;

    
    public AmpIOFalcon(){
        
        motor = new TalonFX(Constants.AMPConstants.canID);
        configs = new TalonFXConfiguration();
        m_request = new MotionMagicVoltage(0);
        slot0 = configs.Slot0;
        slot0.kS = Constants.AMPConstants.kS;
        slot0.kV = Constants.AMPConstants.kV;
        slot0.kA = Constants.AMPConstants.kA;
        slot0.kP = Constants.AMPConstants.kP;
        slot0.kI = Constants.AMPConstants.kI;
        slot0.kD = Constants.AMPConstants.kD;

        configs.MotorOutput.Inverted= InvertedValue.Clockwise_Positive;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        request = new DutyCycleOut(0).withEnableFOC(false);
        motionMagicConfigs = configs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.AMPConstants.motionmagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = Constants.AMPConstants.motionmagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = Constants.AMPConstants.motionmagicJerk;

        motor.getConfigurator().apply(configs);
        
    }
    public void run(double rotationAmount){
        motor.setControl(m_request.withPosition(rotationAmount));
    }
    public void stop(){
        motor.stopMotor();
    }

    

}

