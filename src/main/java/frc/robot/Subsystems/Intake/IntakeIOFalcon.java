package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOFalcon implements IntakeIO {
    private final TalonFX outerMotor;
    private final TalonFX middleMotor;
    private final TalonFX innerMotor;

    private final DutyCycleOut request;

    public IntakeIOFalcon() {
        outerMotor = new TalonFX(IntakeConstants.outerMotorID);
        middleMotor = new TalonFX(IntakeConstants.middleMotorID);
        innerMotor = new TalonFX(IntakeConstants.innerMotorID);

        TalonFXConfiguration outerConfig = new TalonFXConfiguration();
        outerMotor.getConfigurator().apply(outerConfig);
        outerConfig.MotorOutput.Inverted = IntakeConstants.outerMotorInvert;
        outerConfig.MotorOutput.NeutralMode = IntakeConstants.outerMotorBrakeMode;
        outerMotor.getConfigurator().apply(outerConfig);

        TalonFXConfiguration middleConfig = new TalonFXConfiguration();
        middleMotor.getConfigurator().apply(middleConfig);
        middleConfig.MotorOutput.Inverted = IntakeConstants.middleMotorInvert;
        middleConfig.MotorOutput.NeutralMode = IntakeConstants.middleMotorBrakeMode;
        middleMotor.getConfigurator().apply(middleConfig);

        TalonFXConfiguration innerConfig = new TalonFXConfiguration();
        middleMotor.getConfigurator().apply(innerConfig);
        innerConfig.MotorOutput.Inverted = IntakeConstants.innerMotorInvert;
        innerConfig.MotorOutput.NeutralMode = IntakeConstants.innerMotorBrakeMode;
        innerMotor.getConfigurator().apply(innerConfig);

        request = new DutyCycleOut(0).withEnableFOC(true);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.outerMotorPercentOut = outerMotor.getDutyCycle().getValueAsDouble();
        inputs.outerMotorCurrent = outerMotor.getStatorCurrent().getValueAsDouble();
        inputs.outerMotorVelocityRotPerSec = outerMotor.getVelocity().getValueAsDouble();

        inputs.middleMotorPercentOut = middleMotor.getDutyCycle().getValueAsDouble();
        inputs.middleMotorCurrent = middleMotor.getStatorCurrent().getValueAsDouble();
        inputs.middleMotorVelocityRotPerSec = middleMotor.getVelocity().getValueAsDouble();

        inputs.innerMotorPercentOut = innerMotor.getDutyCycle().getValueAsDouble();
        inputs.innerMotorCurrent = innerMotor.getStatorCurrent().getValueAsDouble();
        inputs.innerMotorVelocityRotPerSec = innerMotor.getVelocity().getValueAsDouble();
    }

    public void outerMotorSetPercentOut(double percent) {
        outerMotor.setControl(request.withOutput(percent));
    }

    public void outerMotorStop() {
        outerMotor.stopMotor();
    }

    public void middleMotorSetPercentOut(double percent) {
        middleMotor.setControl(request.withOutput(percent));
    }

    public void middleMotorStop() {
        middleMotor.stopMotor();
    }

    public void innerMotorSetPercentOut(double percent) {
        innerMotor.setControl(request.withOutput(percent));
    }

    public void innerMotorStop() {
        innerMotor.stopMotor();
    }
}
