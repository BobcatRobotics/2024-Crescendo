package frc.robot.Subsystems.Climber;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;

public class ClimberIOFalcon implements ClimberIO {
    private TalonFX climberMotor;
    private DutyCycleOut request;
    private PositionDutyCycle holdPosRequest;

    private StatusSignal<Double> motorStatorCurrent;
    private StatusSignal<Double> motorPosition;

    private double posToHold = 0;

    public ClimberIOFalcon(){
        // The constructor initialized all of the MotionMagic stuff, Inputs, and the motor
        climberMotor = new TalonFX(ClimberConstants.motorID);
        TalonFXConfiguration climberConfigs = new TalonFXConfiguration();
        climberMotor.getConfigurator().apply(climberConfigs);
        climberConfigs.MotorOutput.Inverted = ClimberConstants.climberMotorInvert;
        climberConfigs.MotorOutput.NeutralMode = ClimberConstants.climberMotorBrakeMode;
        climberConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        climberConfigs.CurrentLimits.StatorCurrentLimit = ClimberConstants.currentLimit; //amps
        climberConfigs.Slot0.kP = ClimberConstants.kP;
        climberConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        climberConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.topLimit;
        climberConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        climberConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.bottomLimit;
        climberMotor.getConfigurator().apply(climberConfigs);
        climberMotor.getConfigurator().setPosition(0);

        request = new DutyCycleOut(0).withEnableFOC(true);
        holdPosRequest = new PositionDutyCycle(0).withEnableFOC(true);
        
        motorStatorCurrent = climberMotor.getStatorCurrent();
        motorPosition = climberMotor.getPosition();
        BaseStatusSignal.setUpdateFrequencyForAll(50, motorPosition, motorStatorCurrent);
        climberMotor.optimizeBusUtilization();

        posToHold = motorPosition.getValueAsDouble();
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

    public void holdPos(double rot) {
        climberMotor.setControl(holdPosRequest.withPosition(rot));
    }

    public void stop(){
        // Stops the motor
        climberMotor.stopMotor();
    }


}
