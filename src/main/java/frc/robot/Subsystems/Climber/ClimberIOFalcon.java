package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class ClimberIOFalcon implements ClimberIO
{
    private TalonFX climberMotor;
    private TalonFXConfiguration climberConfigs;
    private MotionMagicVoltage m_voltage;
    private MotionMagicConfigs motionMagicConfigs;
    private MotionMagicDutyCycle climberMotorRequests;


    public ClimberIOFalcon(int deviceID){
        // The constructor initialized all of the MotionMagic stuff, Inputs, and the motor
        climberMotor = new TalonFX(deviceID);
        TalonFXConfiguration climberConfigs = new TalonFXConfiguration();
        climberConfigs.MotorOutput.Inverted = ClimberConstants.climberMotorInvert;
        climberConfigs.MotorOutput.NeutralMode = ClimberConstants.climberMotorBrakeMode;
        climberMotor.setPosition(0);
        climberMotor.getConfigurator().apply(climberConfigs);
        m_voltage = new MotionMagicVoltage(0);
        motionMagicConfigs = climberConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ClimberConstants.motionmagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = ClimberConstants.motionmagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = ClimberConstants.motionmagicJerk;
        this.climberConfigs = climberConfigs;

        //Duty cycle stuff
        climberMotorRequests = new MotionMagicDutyCycle(0).withEnableFOC(true); 
    }

    public void updateInputs(ClimberIOInputs i){
        // Updates all of the inputs/data points being monitored about the motor
        i.climberMotorPercentOut = climberMotor.getDutyCycle().getValueAsDouble();
        i.climberMotorStatorCurrent = climberMotor.getStatorCurrent().getValueAsDouble();
        i.climberMotorVelocityRPS = climberMotor.getVelocity().getValueAsDouble();
        i.climberMotorPosition = climberMotor.getPosition().getValueAsDouble();

        i.motionmagicAcceleration = motionMagicConfigs.MotionMagicAcceleration;
        i.motionmagicCruiseVelocity = motionMagicConfigs.MotionMagicCruiseVelocity;
        i.motionmagicJerk = motionMagicConfigs.MotionMagicJerk;

        if(climberConfigs.MotorOutput.Inverted==InvertedValue.Clockwise_Positive){
            i.climberDirection = "Clockwise";
            ClimberConstants.climberMotorInvert = InvertedValue.Clockwise_Positive;
        }

        if(climberConfigs.MotorOutput.Inverted==InvertedValue.CounterClockwise_Positive){
            i.climberDirection = "Counter-Clockwise";
            ClimberConstants.climberMotorInvert = InvertedValue.CounterClockwise_Positive;
        }

    }

    //This function : you give it a rotation amount and it will run to that rotation amount
    public void run(double rotationAmount){
        // Runs the motor for a certain encoder count using MotionMagic PID
        climberMotor.setControl(m_voltage.withPosition(rotationAmount));
        
        //climberMotor.setControl(climberMotorRequests);
    }

    public void changeVelocity(double newVelocity){
        // Changes the velocity of the motor
        motionMagicConfigs.withMotionMagicCruiseVelocity(newVelocity);
    }

    public void stop(){
        // Stops the motor
        climberMotor.stopMotor();
    }

    public void inverseDirection(){
        // Inverses the direction of the motor in the inputs
        climberConfigs.MotorOutput.Inverted = ClimberConstants.climberMotorInvert;
        climberMotor.getConfigurator().apply(climberConfigs);

    }

}
