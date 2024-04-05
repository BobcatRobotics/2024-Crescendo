package frc.robot.Subsystems.Amp;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.AmpConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

public class AmpIOFalcon implements AmpIO {
    private TalonFX motor;
    private final TalonFXConfiguration configs;
    private final Slot0Configs slot0;
    // private final MotionMagicVoltage m_request;
    private final PositionDutyCycle m_request;
    private final SoftwareLimitSwitchConfigs softLimitThresh;
    private final DutyCycleOut m_dutyCycleOut;
    // private double kP = AmpConstants.kP;
    // private double kI = AmpConstants.kI;
    // private double kD = AmpConstants.kD;
    private final StatusSignal<Double> motorPosition;
    private final StatusSignal<Double> statorCurrent;

    public AmpIOFalcon() {
        softLimitThresh = new SoftwareLimitSwitchConfigs();
        motor = new TalonFX(AmpConstants.canID);
        configs = new TalonFXConfiguration();
        m_request = new PositionDutyCycle(0);
        m_dutyCycleOut = new DutyCycleOut(0);
        slot0 = configs.Slot0;
        
        softLimitThresh.withForwardSoftLimitEnable(false);
        softLimitThresh.withReverseSoftLimitEnable(false);
        softLimitThresh.withForwardSoftLimitThreshold(AmpConstants.forwardSoftLimit);
        softLimitThresh.withReverseSoftLimitThreshold(AmpConstants.reverseSoftLimit);

        slot0.kS = AmpConstants.kS;// initializes  pid values
        slot0.kV = AmpConstants.kV;
        slot0.kA = AmpConstants.kA;
        slot0.kP = AmpConstants.kP;
        slot0.kI = AmpConstants.kI;
        slot0.kD = AmpConstants.kD;
        
        

        configs.MotorOutput.Inverted = AmpConstants.ampInvertedValue;
        configs.MotorOutput.NeutralMode = AmpConstants.ampNeutralModeValue;

        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = AmpConstants.ampStatorCurrentLimit; // amps
                
        //configs.SoftwareLimitSwitch = softLimitThresh;
        //configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        //configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = AmpConstants.forwardSoftLimit; //need to (de)vide by 360 to convert degrees to rotations
        //configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        //configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = AmpConstants.reverseSoftLimit;
        motor.getConfigurator().apply(configs);
        motor.getConfigurator().setPosition(0);
        
        m_request.withEnableFOC(true);

        motorPosition = motor.getPosition();
        statorCurrent = motor.getStatorCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(50, motorPosition, statorCurrent);
        motor.optimizeBusUtilization();
    }

    /*
     * Updates the inputs for the amp subsytem based on the position of the motor,
     * ran periodically
     */
    public void updateInputs(AmpIOInputs inputs) {
        BaseStatusSignal.refreshAll(motorPosition, statorCurrent);
        inputs.motorPosition = (motorPosition.getValueAsDouble()/30)*(360); //degrees
        inputs.current = statorCurrent.getValueAsDouble();
    }

    /*
     * runs the motor to rotiation amount for pid
     */
    public void setPos(double rotationAmount) {
        motor.setControl(m_request.withPosition(rotationAmount*30));//.withFeedForward(0.1));
    }

    public void setPosWithFeedforward(double pos, double ff) {
        motor.setControl(m_request.withPosition(pos*30)); //.withFeedForward(ff));
    }

    /**
     * sets amp motor to a specific percent
     */
    public void setPercent(double percent){
        motor.setControl(m_dutyCycleOut.withOutput(percent));
    }

    /*
     * Stops the motor
     */
    public void stop() {
        motor.stopMotor();
    }

    public void stopMotorFeedforward(){
        //motor.set(0.03);
    }

    public void stopMotorStowPos() {
        motor.set(-0.01);
    }

    public void zeroPosition() {
        motor.getConfigurator().setPosition(0);
    }


} 