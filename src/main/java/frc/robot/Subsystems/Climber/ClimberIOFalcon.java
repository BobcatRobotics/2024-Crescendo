package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
 
import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix6.controls.DutyCycleOut;

public class ClimberIOFalcon implements ClimberIO {
    private TalonFX climberMotor;
    private DutyCycleOut request;

    private StatusSignal<Double> motorStatorCurrent;
    private StatusSignal<Double> motorPosition;

    public ClimberIOFalcon(){
        // The constructor initialized all of the MotionMagic stuff, Inputs, and the motor
        climberMotor = new TalonFX(ClimberConstants.motorID);
        TalonFXConfiguration climberConfigs = new TalonFXConfiguration();
        climberMotor.getConfigurator().apply(climberConfigs);
        climberConfigs.MotorOutput.Inverted = ClimberConstants.climberMotorInvert;
        climberConfigs.MotorOutput.NeutralMode = ClimberConstants.climberMotorBrakeMode;
        climberConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        climberConfigs.CurrentLimits.StatorCurrentLimit = 40; //amps
        climberMotor.getConfigurator().apply(climberConfigs);

        request = new DutyCycleOut(0).withEnableFOC(true);  
        
        motorStatorCurrent = climberMotor.getStatorCurrent();
        motorPosition = climberMotor.getPosition();
        BaseStatusSignal.setUpdateFrequencyForAll(50, motorPosition, motorStatorCurrent);
        climberMotor.optimizeBusUtilization();
    }

    public void updateInputs(ClimberIOInputs inputs){
        BaseStatusSignal.refreshAll(motorStatorCurrent, motorPosition);
        // Updates all of the inputs/data points being monitored about the motor
        inputs.climberMotorStatorCurrent = motorStatorCurrent.getValueAsDouble();
        inputs.climberMotorPosition = motorPosition.getValueAsDouble();
    }

    public void setPercentOut(double percent) {
        climberMotor.setControl(request.withOutput(percent));
    }

    public void stop(){
        // Stops the motor
        climberMotor.stopMotor();
    }

}
