package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
 
import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class ClimberIOFalcon implements ClimberIO
{
    private TalonFX climberMotor;
    private TalonFXConfiguration climberConfigs;
    private MotionMagicVoltage m_voltage;
    private final MotionMagicConfigs motionMagicConfigs;


    public ClimberIOFalcon(int deviceID){
        // The constructor initialized all of the MotionMagic stuff, Inputs, and the motor
        climberMotor = new TalonFX(deviceID);
        TalonFXConfiguration climberConfigs = new TalonFXConfiguration();
        climberMotor.getConfigurator().apply(climberConfigs);
        climberConfigs.MotorOutput.Inverted = ClimberConstants.climberMotorInvert;
        climberMotor.setPosition(0);
        m_voltage = new MotionMagicVoltage(0);
        motionMagicConfigs = climberConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ClimberConstants.motionmagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = ClimberConstants.motionmagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = ClimberConstants.motionmagicJerk;
        climberConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        climberConfigs.CurrentLimits.StatorCurrentLimit = 40; //amps

        climberMotor.getConfigurator().apply(climberConfigs);

        
    }

    public void updateInputs(ClimberIOInputs i){
        // Updates all of the inputs/data points being monitored about the motor
        i.climberMotorStatorCurrent = climberMotor.getStatorCurrent().getValueAsDouble();
        i.climberMotorPosition = climberMotor.getPosition().getValueAsDouble();
    }

    //This function : you give it a rotation amount and it will run to that rotation amount
    public void run(double rotationAmount){
        // Runs the motor for a certain encoder count using MotionMagic PID
        climberMotor.setControl(m_voltage.withPosition(rotationAmount));
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
    }

}
