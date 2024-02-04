package frc.robot.Subsystems.Climber;

import frc.robot.Constants;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class ClimberIOFalcon implements ClimberIO
{
    private TalonFX climberMotor;
    private TalonFXConfiguration climberConfigs;
    private MotionMagicVoltage m_voltage;
    private final MotionMagicConfigs motionMagicConfigs;


    public ClimberIOFalcon(int deviceID){
        climberMotor = new TalonFX(deviceID);
        TalonFXConfiguration climberConfigs = new TalonFXConfiguration();
        climberMotor.getConfigurator().apply(climberConfigs);
        climberConfigs.MotorOutput.Inverted = Constants.climberConstants.climberMotorInvert;
        climberMotor.setPosition(0);
        m_voltage = new MotionMagicVoltage(0);
        motionMagicConfigs = climberConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.climberConstants.motionmagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = Constants.climberConstants.motionmagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = Constants.climberConstants.motionmagicJerk;
        this.climberConfigs = climberConfigs;
    }

    public void updateInputs(ClimberIOInputs i){
        i.climberMotorPercentOut = climberMotor.getDutyCycle().getValueAsDouble();
        i.climberMotorStatorCurrent = climberMotor.getStatorCurrent().getValueAsDouble();
        i.climberMotorVelocityRPS = climberMotor.getVelocity().getValueAsDouble();
        i.climberMotorPosition = climberMotor.getPosition().getValueAsDouble();

        i.motionmagicAcceleration = motionMagicConfigs.MotionMagicAcceleration;
        i.motionmagicCruiseVelocity = motionMagicConfigs.MotionMagicCruiseVelocity;
        i.motionmagicJerk = motionMagicConfigs.MotionMagicJerk;
    }

    //This function : you give it a rotation amount and it will run to that rotation amount
    public void run(double rotationAmount){
        climberMotor.setControl(m_voltage.withPosition(rotationAmount));
        
    }

    public void changeVelocity(double newVelocity){
        motionMagicConfigs.withMotionMagicCruiseVelocity(newVelocity);
    }

    public void stop(){
        climberMotor.stopMotor();
    }

    public void inverseDirection(){
        climberConfigs.MotorOutput.Inverted = Constants.climberConstants.climberMotorInvert;
    }

}
