package frc.robot.Subsystems.Trap;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.TrapConstants;

import com.ctre.phoenix6.controls.DutyCycleOut;

public class TrapIOFalcon implements TrapIO {
    private TalonFX armMotor;
    private TalonFX rollerMotor;

    private DutyCycleOut armRequest;
    private DutyCycleOut rollerRequest;

    private StatusSignal<Double> position;

    public TrapIOFalcon() {
        armMotor = new TalonFX(TrapConstants.armID);       
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        armMotor.getConfigurator().apply(armConfig);
        armConfig.MotorOutput.Inverted = TrapConstants.armInvert;
        armConfig.MotorOutput.NeutralMode = TrapConstants.armBrakeMode;
        armMotor.getConfigurator().apply(armConfig);

        rollerMotor = new TalonFX(TrapConstants.rollerID);       
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerMotor.getConfigurator().apply(rollerConfig);
        rollerConfig.MotorOutput.Inverted = TrapConstants.rollerInvert;
        rollerConfig.MotorOutput.NeutralMode = TrapConstants.rollerBrakeMode;
        rollerMotor.getConfigurator().apply(rollerConfig);

        armRequest = new DutyCycleOut(0).withEnableFOC(true);
        rollerRequest = new DutyCycleOut(0).withEnableFOC(true);

        position = armMotor.getPosition();
        armMotor.optimizeBusUtilization();
        rollerMotor.optimizeBusUtilization();
    }

    public void updateInputs(TrapIOInputs inputs) {
        BaseStatusSignal.refreshAll(position);
        inputs.trapPosition = position.getValueAsDouble()*360;
    } 

    public void setArmPercent(double percent) {
        armMotor.setControl(armRequest.withOutput(percent));
    }

    @Override
    public void setRollerPercent(double percent) {
        rollerMotor.setControl(rollerRequest.withOutput(percent));
    }

    @Override
    public void stopArm() {
        armMotor.stopMotor();
    }

    @Override
    public void stopRollers() {
        rollerMotor.stopMotor();
    }
}
