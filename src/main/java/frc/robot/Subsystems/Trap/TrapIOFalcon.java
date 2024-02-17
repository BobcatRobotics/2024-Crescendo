package frc.robot.Subsystems.Trap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants.TrapConstants;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class TrapIOFalcon {
    private TalonFX winchMotor;
    private TalonFX shooterMotor;

    private final VelocityDutyCycle requestShooter; 
    private final MotionMagicDutyCycle requestWinch;
    private final VelocityVoltage voltageRequest;

    
    private TalonFXConfiguration winchConfigs;
    private TalonFXConfiguration shooterConfigs;

   private MotionMagicConfigs motionMagicConfigs;
   private MotionMagicVoltage m_voltage;

    public TrapIOFalcon(int deviceIDWinch, int deviceIDShooter){

        // General initialization of the motors amd default configs
        winchMotor = new TalonFX(deviceIDWinch);
        shooterMotor = new TalonFX(deviceIDShooter);

        // Apply default configs
        winchConfigs = new TalonFXConfiguration();
        winchMotor.getConfigurator().apply(winchConfigs);
        shooterConfigs = new TalonFXConfiguration();
        shooterMotor.getConfigurator().apply(shooterConfigs);

        // Invert and brake mode
        winchConfigs.MotorOutput.Inverted = TrapConstants.winchMotorInvert;
        winchConfigs.MotorOutput.NeutralMode = TrapConstants.winchMotorBrakeMode; 
        shooterConfigs.MotorOutput.NeutralMode = TrapConstants.shooterMotorBrakeMode;
        shooterConfigs.MotorOutput.Inverted = TrapConstants.shooterMotorInvert;

        // Motion Magic initialization just for the shoulder motor
        m_voltage = new MotionMagicVoltage(0);
        motionMagicConfigs = winchConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = TrapConstants.motionmagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = TrapConstants.motionmagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = TrapConstants.motionmagicJerk;
        winchMotor.setPosition(0);

        // Non motion magic properties of the shooter motor
        shooterConfigs.Slot0.kP = TrapConstants.K;
        shooterConfigs.Slot0.kV = TrapConstants.V;
        shooterConfigs.Slot0.kS = TrapConstants.S;

        // Duty Cycle
        requestWinch = new MotionMagicDutyCycle(0).withEnableFOC(true);
        requestShooter =  new VelocityDutyCycle(0).withEnableFOC(true);
        voltageRequest = new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    }

}
