package frc.robot.Subsystems.Trap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.TrapConstants;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class TrapIOFalcon {
    private TalonFX winchMotor;
    private TalonFX shooterMotor;
    
    private TalonFXConfiguration winchConfigs;
    private TalonFXConfiguration shooterConfigs;

   private MotionMagicConfigs motionMagicConfigs;
   private MotionMagicVoltage m_voltage;

    public TrapIOFalcon(int deviceIDWinch, int deviceIDShooter){

        // General initialization of the motors
        winchMotor = new TalonFX(deviceIDWinch);
        shooterMotor = new TalonFX(deviceIDShooter);
        winchConfigs = new TalonFXConfiguration();
        winchMotor.getConfigurator().apply(winchConfigs);
        shooterConfigs = new TalonFXConfiguration();
        shooterMotor.getConfigurator().apply(shooterConfigs);

        // Motion Magic initialization just for the shoulder motor
        m_voltage = new MotionMagicVoltage(0);
        motionMagicConfigs = winchConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = TrapConstants.motionmagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = TrapConstants.motionmagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = TrapConstants.motionmagicJerk;



    }

}
