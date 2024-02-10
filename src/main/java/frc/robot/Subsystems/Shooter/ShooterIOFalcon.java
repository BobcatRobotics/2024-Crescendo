package frc.robot.Subsystems.Shooter;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.FeedbackConfigs;

import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;

public class ShooterIOFalcon implements ShooterIO {
    private TalonFX topMotor;
    private TalonFX bottomMotor;
    private TalonFX angleMotor;
    private TalonFX feederMotor;
    private final Slot0Configs angleMotor_slot0;
    private final Slot0Configs topMotor_slot0;
    private final Slot0Configs bottomMotor_slot0;
    private final Slot0Configs feederMotor_slot0;
    private final MotionMagicConfigs angleMotorMotionMagicConfigs;
    private final MotionMagicConfigs feederMotorMotionMagicConfigs;
    private final MotionMagicConfigs topMotorMotionMagicConfigs;
    private final MotionMagicConfigs bottomMotorMotionMagicConfigs;
    private final MotionMagicVoltage m_request;    

    private final DutyCycleOut request; //same as percent out
    
    public ShooterIOFalcon() {
        m_request = new MotionMagicVoltage(0);


        topMotor = new TalonFX(Constants.shooterConstants.topMotorID); //initializes TalonFX motor 1
        bottomMotor = new TalonFX(Constants.shooterConstants.bottomMotorID); //initializes TalonFX motor 2
        angleMotor = new TalonFX(Constants.shooterConstants.angleMotorID); //initializes TalonFX motor 3
        

        TalonFXConfiguration topConfigs = new TalonFXConfiguration();
        topMotor.getConfigurator().apply(topConfigs);
        topConfigs.MotorOutput.Inverted = shooterConstants.topMotorInvert;
        topConfigs.MotorOutput.NeutralMode = shooterConstants.topMotorBrakeMode;
        //Motion Magic configs
        topMotor_slot0 = topConfigs.Slot0;
        topMotorMotionMagicConfigs = topConfigs.MotionMagic;


        TalonFXConfiguration bottomConfigs = new TalonFXConfiguration();
        bottomMotor.getConfigurator().apply(bottomConfigs);
        bottomConfigs.MotorOutput.Inverted = shooterConstants.bottomMotorInvert;
        bottomConfigs.MotorOutput.NeutralMode = shooterConstants.bottomMotorBrakeMode;
        //Motion Magic configs
        bottomMotor_slot0 = bottomConfigs.Slot0;
        bottomMotorMotionMagicConfigs = bottomConfigs.MotionMagic;

        TalonFXConfiguration angleConfigs = new TalonFXConfiguration();
        angleMotor.getConfigurator().apply(angleConfigs);
        angleConfigs.MotorOutput.Inverted = shooterConstants.angleMotorInvert;
        angleConfigs.MotorOutput.NeutralMode = shooterConstants.angleMotorBrakeMode;
        //Motion Magic configs
        angleMotor_slot0 = angleConfigs.Slot0;
        angleMotorMotionMagicConfigs = angleConfigs.MotionMagic;
        angleMotor_slot0.kS = Constants.shooterConstants.kAngleS;
        angleMotor_slot0.kV = Constants.shooterConstants.kAngleV;
        angleMotor_slot0.kA = Constants.shooterConstants.kAngleA;
        angleMotor_slot0.kP = Constants.shooterConstants.kAngleP;
        angleMotor_slot0.kI = Constants.shooterConstants.kAngleI;
        angleMotor_slot0.kD = Constants.shooterConstants.kAngleD;
       // angleMotorMotionMagicConfigs.MotionMagicCruiseVelocity = Constants.shooterConstants.angleMotorMotionMagicCruiseVelocity;
       // angleMotorMotionMagicConfigs.MotionMagicAcceleration = Constants.shooterConstants.angleMotorMotionMagicAcceleration;
       // angleMotorMotionMagicConfigs.MotionMagicJerk = Constants.shooterConstants.angleMotorMotionMagicJerk;

        TalonFXConfiguration feederConfigs = new TalonFXConfiguration();
        feederMotor.getConfigurator().apply(feederConfigs);
        
        //Motion Magic configs
        feederMotor_slot0 = feederConfigs.Slot0;
        feederMotorMotionMagicConfigs = feederConfigs.MotionMagic;


        request = new DutyCycleOut(0).withEnableFOC(true);
    }

    public void updateInputs(ShooterIOInputs i) {
        //i.topMotorPercentOut = topMotor.getDutyCycle().getValueAsDouble();
        i.topMotorStatorCurrent = topMotor.getStatorCurrent().getValueAsDouble();
        i.topMotorVelocityRPS = topMotor.getVelocity().getValueAsDouble();
        
       // i.bottomMotorPercentOut = bottomMotor.getDutyCycle().getValueAsDouble();
        i.bottomMotorStatorCurrent = bottomMotor.getStatorCurrent().getValueAsDouble();
        i.bottomMotorVelocityRPS = bottomMotor.getVelocity().getValueAsDouble();
        
       // i.angleMotorPercentOut = angleMotor.getDutyCycle().getValueAsDouble();
        i.angleMotorStatorCurrent = angleMotor.getStatorCurrent().getValueAsDouble();
       // i.angleMotorVelocityRPS = angleMotor.getVelocity().getValueAsDouble();
        
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


    public void setBothMotorsRPM(double topRPM, double bottomRPM){
        topMotor.setControl(m_request);
    }

}