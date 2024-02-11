package frc.robot.Subsystems.Shooter;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOFalcon implements ShooterIO {
    private TalonFX topMotor; //this will control the top rollers in the shooter "arm"
    private TalonFX bottomMotor; //this controls the bottom rollers of the shooter arm
    private TalonFX angleMotor; //this is responsbile for the angle of the shooter
    
    //Here lies "feederMotor" :(      R.I.P. 2024-2024; a wonderful son, brother, and father.

    private CANcoder cancoder;

    private final VelocityDutyCycle requestTop; 
    private final VelocityDutyCycle requestBottom; 
    private final MotionMagicDutyCycle angleRequest;
    private final VelocityVoltage voltageRequest;
    
    public ShooterIOFalcon() {
        topMotor = new TalonFX(ShooterConstants.topMotorID); //initializes TalonFX motor 1
        bottomMotor = new TalonFX(ShooterConstants.bottomMotorID); //initializes TalonFX motor 2
        angleMotor = new TalonFX(ShooterConstants.angleMotorID); //initializes TalonFX motor 3
        cancoder = new CANcoder(ShooterConstants.cancoderID);


        //Top motor configurations
        TalonFXConfiguration topConfigs = new TalonFXConfiguration();
        topMotor.getConfigurator().apply(topConfigs); //reset to default
        topConfigs.MotorOutput.Inverted = ShooterConstants.topMotorInvert;
        topConfigs.MotorOutput.NeutralMode = ShooterConstants.topMotorBrakeMode;
        topConfigs.Slot0.kP = ShooterConstants.kTopP;
        topConfigs.Slot0.kV = ShooterConstants.kTopV;
        topConfigs.Slot0.kS = ShooterConstants.kTopS;
        // topConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        // topConfigs.CurrentLimits.StatorCurrentLimit = 40;
        topMotor.getConfigurator().apply(topConfigs); //Apply custom configs

        //Bottom motor configurations
        TalonFXConfiguration bottomConfigs = new TalonFXConfiguration();
        bottomMotor.getConfigurator().apply(bottomConfigs);
        bottomConfigs.MotorOutput.Inverted = ShooterConstants.bottomMotorInvert;
        bottomConfigs.MotorOutput.NeutralMode = ShooterConstants.bottomMotorBrakeMode;
        bottomConfigs.Slot0.kP = ShooterConstants.kBottomP;
        bottomConfigs.Slot0.kV = ShooterConstants.kBottomV;
        bottomConfigs.Slot0.kS = ShooterConstants.kBottomS;
        // bottomConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        // bottomConfigs.CurrentLimits.StatorCurrentLimit = 40;
        bottomMotor.getConfigurator().apply(bottomConfigs);

        //Angle motor configurations
        TalonFXConfiguration angleConfigs = new TalonFXConfiguration();
        angleMotor.getConfigurator().apply(angleConfigs);
        angleConfigs.MotorOutput.Inverted = ShooterConstants.angleMotorInvert;
        angleConfigs.MotorOutput.NeutralMode = ShooterConstants.angleMotorBrakeMode;
        angleConfigs.Slot0.kP = ShooterConstants.kAngleP;
        angleConfigs.Slot0.kI = ShooterConstants.kAngleI;
        angleConfigs.Slot0.kD = ShooterConstants.kAngleD;
        angleConfigs.Slot0.kS = ShooterConstants.kAngleS;
        angleConfigs.Slot0.kV = ShooterConstants.kAngleV;
        angleConfigs.Slot0.kA = ShooterConstants.kAngleA;
        angleConfigs.Feedback.FeedbackRemoteSensorID = ShooterConstants.cancoderID;
        angleConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        angleConfigs.Feedback.RotorToSensorRatio = ShooterConstants.rotorToSensorRatio;
        angleConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        angleConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShooterConstants.topLimit/360;
        angleConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        angleConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ShooterConstants.bottomLimit/360;
        angleConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        angleConfigs.CurrentLimits.StatorCurrentLimit = 40;
        angleMotor.getConfigurator().apply(angleConfigs);


        //CANcoder configurations
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoder.getConfigurator().apply(cancoderConfigs);
        cancoderConfigs.MagnetSensor.SensorDirection = ShooterConstants.sensorDirection;
        cancoderConfigs.MagnetSensor.AbsoluteSensorRange = ShooterConstants.sensorRange;
        cancoderConfigs.MagnetSensor.MagnetOffset = ShooterConstants.offset.getRotations();
        cancoder.getConfigurator().apply(cancoderConfigs);


        //Updates the requests for each motor
        requestTop = new VelocityDutyCycle(0).withEnableFOC(true);
        requestBottom = new VelocityDutyCycle(0).withEnableFOC(true);
        angleRequest = new MotionMagicDutyCycle(0).withEnableFOC(true);
        voltageRequest = new VelocityVoltage(0).withEnableFOC(true).withSlot(0);

    }

    /**
     * 
     */
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topMotorStatorCurrent = topMotor.getStatorCurrent().getValueAsDouble();
        inputs.topMotorVelocityRPS = topMotor.getVelocity().getValueAsDouble();
        
        inputs.bottomMotorStatorCurrent = bottomMotor.getStatorCurrent().getValueAsDouble();
        inputs.bottomMotorVelocityRPS = bottomMotor.getVelocity().getValueAsDouble();
        
        inputs.angleMotorPosition = angleMotor.getPosition().getValueAsDouble();
        inputs.angleMotorStatorCurrent = angleMotor.getStatorCurrent().getValueAsDouble();
    }

    /**
     * @param rps in revs per second.......
     */
    public void setTopVelocity(double rps) {
        topMotor.setControl(requestTop.withVelocity(rps));
    }

    /**
     * @param rps also in revs per second.....
     */
    public void setBottomVelocity(double rps) {
        bottomMotor.setControl(requestBottom.withVelocity(rps));
    }

    /**
     * @param deg also in revs per second...haha just kidding its in degrees
     */
    public void setAngle(double deg) {
        angleMotor.setControl(angleRequest.withPosition(Rotation2d.fromDegrees(deg).getRotations()));
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

    public void setVelocityTune(double rpm){
        double rps = rpm/60;
        topMotor.setControl(voltageRequest.withVelocity(rps));
        bottomMotor.setControl(voltageRequest.withVelocity(rps));
    }


}