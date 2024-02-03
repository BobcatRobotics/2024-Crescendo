package frc.robot.Subsystems.Climber;
import com.ctre.phoenix6.jni.CtreJniWrapper;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterIOInputs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.hardware.TalonFX;


public class ClimberIOFalcon implements ClimberIO
{
    private TalonFX climberMotor;

    public ClimberIOFalcon(int deviceID){
        climberMotor = new TalonFX(deviceID);
        TalonFXConfiguration climberConfigs = new TalonFXConfiguration();
        climberMotor.getConfigurator().apply(climberConfigs);
        climberConfigs.MotorOutput.Inverted = Constants.climberConstants.climberMotorInvert;
        climberMotor.setPosition(0);
    }

    public void updateInputs(ClimberIOInputs i){
        i.climberMotorPercentOut = climberMotor.getDutyCycle().getValueAsDouble();
        i.climberMotorStatorCurrent = climberMotor.getStatorCurrent().getValueAsDouble();
        i.climberMotorVelocityRPS = climberMotor.getVelocity().getValueAsDouble();
        i.climberMotorPosition = climberMotor.getPosition().getValueAsDouble();
    }

    public void setClimberSpeed(){

    }

}
