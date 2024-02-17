package frc.robot.Subsystems.Amp;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants.AmpConstants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

public class AmpIOFalcon implements AmpIO {
    private TalonFX motor;
    private final TalonFXConfiguration configs;
    private final Slot0Configs slot0;
    private final MotionMagicConfigs motionMagicConfigs;
    // private final MotionMagicVoltage m_request;
    private final MotionMagicVoltage m_request;
    private final SoftwareLimitSwitchConfigs softLimitThresh;
    // private double kP = AmpConstants.kP;
    // private double kI = AmpConstants.kI;
    // private double kD = AmpConstants.kD;

    public AmpIOFalcon() {
        softLimitThresh = new SoftwareLimitSwitchConfigs();
        motor = new TalonFX(AmpConstants.canID);
        configs = new TalonFXConfiguration();
        m_request = new MotionMagicVoltage(0);
        slot0 = configs.Slot0;
        softLimitThresh.withForwardSoftLimitEnable(true);
        softLimitThresh.withReverseSoftLimitEnable(true);
        softLimitThresh.withForwardSoftLimitThreshold(AmpConstants.forwardsoftlimit);
        softLimitThresh.withReverseSoftLimitThreshold(AmpConstants.reversesoftlimit5);

        slot0.kS = AmpConstants.kS;// initializes motion magic pid values
        slot0.kV = AmpConstants.kV;
        slot0.kA = AmpConstants.kA;
        slot0.kP = AmpConstants.kP;
        slot0.kI = AmpConstants.kI;
        slot0.kD = AmpConstants.kD;

        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = 40; // amps

        m_request.withEnableFOC(true);
        motionMagicConfigs = configs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = AmpConstants.motionmagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = AmpConstants.motionmagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = AmpConstants.motionmagicJerk;

        configs.SoftwareLimitSwitch = softLimitThresh;
        motor.getConfigurator().apply(configs);

    }

    /*
     * Updates the inputs for the amp subsytem based on the position of the motor,
     * ran periodically
     */
    public void updateInputs(AmpIOInputs inputs) {
        inputs.motorposition = motor.getPosition().getValueAsDouble();
    }

    /*
     * runs the motor to rotiation amount for pid
     */
    public void setPos(double rotationAmount) {
        motor.setControl(m_request.withPosition(rotationAmount));
    }

    /*
     * Stops the motor
     */
    public void stop() {
        motor.stopMotor();
    }

}
