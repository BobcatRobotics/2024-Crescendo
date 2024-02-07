package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOFalcon implements IntakeIO {
    private final TalonFX switchMotor;
    private final TalonFX floorMotor;
    private final TalonFX outsideMotor;

    // private final TimeOfFlight tof;

    private final DutyCycleOut request;

    public IntakeIOFalcon() {
        switchMotor = new TalonFX(IntakeConstants.switchMotorID);
        floorMotor = new TalonFX(IntakeConstants.floorMotorID);
        outsideMotor = new TalonFX(IntakeConstants.outsideMotorID);

        // tof = new TimeOfFlight(IntakeConstants.tofID);

        TalonFXConfiguration switchConfig = new TalonFXConfiguration();
        switchMotor.getConfigurator().apply(switchConfig);
        switchConfig.MotorOutput.Inverted = IntakeConstants.switchMotorInvert;
        switchConfig.MotorOutput.NeutralMode = IntakeConstants.switchMotorBrakeMode;
        switchMotor.getConfigurator().apply(switchConfig);

        TalonFXConfiguration floorConfig = new TalonFXConfiguration();
        floorMotor.getConfigurator().apply(floorConfig);
        floorConfig.MotorOutput.Inverted = IntakeConstants.floorMotorInvert;
        floorConfig.MotorOutput.NeutralMode = IntakeConstants.floorMotorBrakeMode;
        floorMotor.getConfigurator().apply(floorConfig);

        TalonFXConfiguration outsideConfig = new TalonFXConfiguration();
        floorMotor.getConfigurator().apply(outsideConfig);
        outsideConfig.MotorOutput.Inverted = IntakeConstants.outsideMotorInvert;
        outsideConfig.MotorOutput.NeutralMode = IntakeConstants.outsideMotorBrakeMode;
        outsideMotor.getConfigurator().apply(outsideConfig);

        request = new DutyCycleOut(0).withEnableFOC(true);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.switchMotorPercentOut = switchMotor.getDutyCycle().getValueAsDouble();
        inputs.switchMotorCurrent = switchMotor.getStatorCurrent().getValueAsDouble();
        inputs.switchMotorVelocityRotPerSec = switchMotor.getVelocity().getValueAsDouble();

        inputs.floorMotorPercentOut = floorMotor.getDutyCycle().getValueAsDouble();
        inputs.floorMotorCurrent = floorMotor.getStatorCurrent().getValueAsDouble();
        inputs.floorMotorVelocityRotPerSec = floorMotor.getVelocity().getValueAsDouble();

        inputs.outsideMotorPercentOut = outsideMotor.getDutyCycle().getValueAsDouble();
        inputs.outsideMotorCurrent = outsideMotor.getStatorCurrent().getValueAsDouble();
        inputs.outsideMotorVelocityRotPerSec = outsideMotor.getVelocity().getValueAsDouble();
    }

    public void switchMotorSetPercentOut(double percent) {
        switchMotor.setControl(request.withOutput(percent));
    }

    public void switchMotorStop() {
        switchMotor.stopMotor();
    }

    public void floorMotorSetPercentOut(double percent) {
        floorMotor.setControl(request.withOutput(percent));
    }

    public void floorMotorStop() {
        floorMotor.stopMotor();
    }

    public void outsideMotorSetPercentOut(double percent) {
        outsideMotor.setControl(request.withOutput(percent));
    }

    public void outsideMotorStop() {
        outsideMotor.stopMotor();
    }
}
