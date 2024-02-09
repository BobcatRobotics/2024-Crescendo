package frc.robot.Subsystems.Trap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants.TrapConstants;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class TrapIOFalcon {
    private TalonFX winchMotor;
    private TalonFX shooterMotor;
    
    private TalonFXConfiguration winchConfigs;
    private TalonFXConfiguration shooterConfigs;

   // private final MotionMagicConfigs motionMagicConfigs;

    public TrapIOFalcon(int deviceIDWinch, int deviceIDShooter){
        winchMotor = new TalonFX(deviceIDWinch);
        shooterMotor = new TalonFX(deviceIDShooter);

        winchConfigs = new TalonFXConfiguration();
        winchMotor.getConfigurator().apply(winchConfigs);

        shooterConfigs = new TalonFXConfiguration();
        shooterMotor.getConfigurator().apply(shooterConfigs);
    }

}
