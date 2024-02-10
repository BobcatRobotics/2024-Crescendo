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


public class AmpIO implements Amp{
    private TalonFX motor;
    private final TalonFXConfiguration configs;
    private final DutyCycleOut request;
    private double kP = 75e-4; 
    private double kI = 0;
    private double kD = 0;

    
    public AmpIO(){
        
        motor = new TalonFX(Constants.AmpConstants);
        configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted= InvertedValue.Clockwise_Positive;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(configs);
        request = new DutyCycleOut(0).withEnableFOC(false);
        

    }
}
