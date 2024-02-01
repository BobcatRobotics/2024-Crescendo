package frc.robot.Subsystems.AmpShooter;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ShooterIOFalcon implements ShooterIO {
    private TalonFX motor;
    private TalonFX motor2;
    // private final TalonFXConfiguration configs;
    // private final DutyCycleOut request; //same as percent out


    // public ShooterIOFalcon() {
    //     motor = new TalonFX((int) (Constants.ShooterConstants.shooterMotorID1)); //initializes TalonFX motor
    //     motor2 = new TalonFX((int) (Constants.ShooterConstants.shooterMotorID2)); //initializes TalonFX motor
    // }

    public void updateInputs(ShooterIOInputs i) {
        i.percentOut = motor.getDutyCycle().getValueAsDouble();
        i.statorCurrent = motor.getStatorCurrent().getValueAsDouble();
        i.velocityRPS = motor.getVelocity().getValueAsDouble();
    }

    public void updateConfigs() {
        //yap yap
    }

    public void setPercentOUt(double in) {
    }
}
