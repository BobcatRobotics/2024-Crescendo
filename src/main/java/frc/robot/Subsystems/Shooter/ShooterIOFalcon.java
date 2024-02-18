package frc.robot.Subsystems.Shooter;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOFalcon implements ShooterIO {
    private TalonFX topMotor; //this will control the top rollers in the shooter "arm"
    private TalonFX bottomMotor; //this controls the bottom rollers of the shooter arm
    
    //Here lies "feederMotor" :(      R.I.P. 2024-2024; a wonderful son, brother, and father.


    private final VelocityDutyCycle requestTop; 
    private final VelocityDutyCycle requestBottom; 
        
    private final VelocityDutyCycle voltageRequestTop;
    private final VelocityDutyCycle voltageRequestBottom;
    
    public ShooterIOFalcon() {
        topMotor = new TalonFX(ShooterConstants.topMotorID); //initializes TalonFX motor 1
        bottomMotor = new TalonFX(ShooterConstants.bottomMotorID); //initializes TalonFX motor 2


        //Top motor configurations
        TalonFXConfiguration topConfigs = new TalonFXConfiguration();
        topMotor.getConfigurator().apply(topConfigs); //reset to default
        topConfigs.MotorOutput.Inverted = ShooterConstants.topMotorInvert;
        topConfigs.MotorOutput.NeutralMode = ShooterConstants.topMotorBrakeMode;
        topConfigs.Slot0.kP = ShooterConstants.kTopP;
        topConfigs.Slot0.kV = ShooterConstants.kTopV;
        topConfigs.Slot0.kS = ShooterConstants.kTopS;
        topConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        topConfigs.CurrentLimits.StatorCurrentLimit = 40;
        
        //Bottom motor configurations
        TalonFXConfiguration bottomConfigs = new TalonFXConfiguration();
        bottomMotor.getConfigurator().apply(bottomConfigs);
        bottomConfigs.MotorOutput.Inverted = ShooterConstants.bottomMotorInvert;
        bottomConfigs.MotorOutput.NeutralMode = ShooterConstants.bottomMotorBrakeMode;
        bottomConfigs.Slot0.kP = ShooterConstants.kBottomP;
        bottomConfigs.Slot0.kV = ShooterConstants.kBottomV;
        bottomConfigs.Slot0.kS = ShooterConstants.kBottomS;
        bottomConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        bottomConfigs.CurrentLimits.StatorCurrentLimit = 40;

        bottomMotor.getConfigurator().apply(bottomConfigs);




        


        //Updates the requests for each motor
        requestTop = new VelocityDutyCycle(0).withEnableFOC(true);
        requestBottom = new VelocityDutyCycle(0).withEnableFOC(true);
        voltageRequestTop = new VelocityDutyCycle(0).withEnableFOC(true);
        voltageRequestBottom = new VelocityDutyCycle(0).withEnableFOC(true);
    }


    /**
     * 
     */
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topMotorStatorCurrent = topMotor.getStatorCurrent().getValueAsDouble();
        inputs.topMotorVelocityRPS = topMotor.getVelocity().getValueAsDouble();
        
        inputs.bottomMotorStatorCurrent = bottomMotor.getStatorCurrent().getValueAsDouble();
        inputs.bottomMotorVelocityRPS = bottomMotor.getVelocity().getValueAsDouble();
        
    }

    /**
     * @param rps revs per second
     */
    public void setTopVelocity(double rps) {
        topMotor.setControl(requestTop.withVelocity(rps));
    }


    /**
     * @param rps revs per second
     */
    public void setBottomVelocity(double rps) {
        bottomMotor.setControl(requestBottom.withVelocity(rps));
    }



    /**
     * 
     * @return returns a double in RPS
     */
    public double getTopVelocity() {
        return topMotor.getVelocity().getValueAsDouble(); //rps
    }

    /**
     * 
     * @return returns a double in RPS
     */
    public double getBottomVelocity() {
        return bottomMotor.getVelocity().getValueAsDouble(); //rps
    }



    public void stopTopMotor() {
        topMotor.stopMotor();
    }

    public void stopBottomMotor() {
        bottomMotor.stopMotor();
    }



    /**
     * testing only
     */
    public void setVelocity(double rpm){
        double rps = rpm/60;
        topMotor.setControl(voltageRequestTop.withVelocity(rps));
        bottomMotor.setControl(voltageRequestBottom.withVelocity(rps));
    }






}