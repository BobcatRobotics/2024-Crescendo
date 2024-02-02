package frc.robot.Subsystems.Shooter;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;;

public class ShooterIOFalcon implements ShooterIO {
    private TalonFX topMotor;
    private TalonFX bottomMotor;
    private TalonFX angleMotor;
    private TalonFX feederMotor;

    private final DutyCycleOut request; //same as percent out
    
    public ShooterIOFalcon() {
        topMotor = new TalonFX(Constants.shooterConstants.topMotorID); //initializes TalonFX motor 1
        bottomMotor = new TalonFX(Constants.shooterConstants.bottomMotorID); //initializes TalonFX motor 2
        angleMotor = new TalonFX(Constants.shooterConstants.angleMotorID); //initializes TalonFX motor 3
        feederMotor = new TalonFX(Constants.shooterConstants.feederMotorID); //initializes TalonFX motor 3

        TalonFXConfiguration topConfigs = new TalonFXConfiguration();
        topMotor.getConfigurator().apply(topConfigs);
        topConfigs.MotorOutput.Inverted = shooterConstants.topMotorInvert;
        topConfigs.MotorOutput.NeutralMode = shooterConstants.topMotorBrakeMode;

        TalonFXConfiguration bottomConfigs = new TalonFXConfiguration();
        bottomMotor.getConfigurator().apply(bottomConfigs);
        bottomConfigs.MotorOutput.Inverted = shooterConstants.bottomMotorInvert;
        bottomConfigs.MotorOutput.NeutralMode = shooterConstants.bottomMotorBrakeMode;

        TalonFXConfiguration angleConfigs = new TalonFXConfiguration();
        angleMotor.getConfigurator().apply(angleConfigs);
        angleConfigs.MotorOutput.Inverted = shooterConstants.angleMotorInvert;
        angleConfigs.MotorOutput.NeutralMode = shooterConstants.angleMotorBrakeMode;

        TalonFXConfiguration feederConfigs = new TalonFXConfiguration();
        feederMotor.getConfigurator().apply(feederConfigs);
        feederConfigs.MotorOutput.Inverted = shooterConstants.feederMotorInvert;
        feederConfigs.MotorOutput.NeutralMode = shooterConstants.feederMotorBrakeMode;

        request = new DutyCycleOut(0).withEnableFOC(true);
    }

    public void updateInputs(ShooterIOInputs i) {
        i.topMotorPercentOut = topMotor.getDutyCycle().getValueAsDouble();
        i.topMotorStatorCurrent = topMotor.getStatorCurrent().getValueAsDouble();
        i.topMotorVelocityRPS = topMotor.getVelocity().getValueAsDouble();
        
        i.bottomMotorPercentOut = bottomMotor.getDutyCycle().getValueAsDouble();
        i.bottomMotorStatorCurrent = bottomMotor.getStatorCurrent().getValueAsDouble();
        i.bottomMotorVelocityRPS = bottomMotor.getVelocity().getValueAsDouble();
        
        i.angleMotorPercentOut = angleMotor.getDutyCycle().getValueAsDouble();
        i.angleMotorStatorCurrent = angleMotor.getStatorCurrent().getValueAsDouble();
        i.angleMotorVelocityRPS = angleMotor.getVelocity().getValueAsDouble();
        
        i.feederMotorPercentOut = feederMotor.getDutyCycle().getValueAsDouble();
        i.feederMotorStatorCurrent = feederMotor.getStatorCurrent().getValueAsDouble();
        i.feederMotorVelocityRPS = feederMotor.getVelocity().getValueAsDouble();
    }

    public void updateConfigs() {

    }

    public void setTopPercentOut(double pct) {
        topMotor.setControl(request.withOutput(pct).withEnableFOC(true));
    }

    public void setBottomPercentOut(double pct) {
        bottomMotor.setControl(request.withOutput(pct).withEnableFOC(true));
    }

    public void setFeederMotorPercentOut(double pct) {
        feederMotor.setControl(request.withOutput(pct).withEnableFOC(true));
    }

    public void setAngleMotorPercentOut(double pct) {
        angleMotor.setControl(request.withOutput(pct).withEnableFOC(true));
    }


    public void stopTopMotor() {
        topMotor.stopMotor();
    }

    public void stopBottomMotor() {
        bottomMotor.stopMotor();
    }

    public void stopAngleMotor() {
        angleMotor.stopMotor();
    }

    public void stopFeederMotor() {
        feederMotor.stopMotor();
    }

}