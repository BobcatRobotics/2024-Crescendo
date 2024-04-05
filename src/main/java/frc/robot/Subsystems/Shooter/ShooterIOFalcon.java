package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import frc.robot.Constants.ShooterConstants;

public class ShooterIOFalcon implements ShooterIO {
    private TalonFX topMotor; // this will control the top rollers in the shooter "arm"
    private TalonFX bottomMotor; // this controls the bottom rollers of the shooter arm

    // Here lies "feederMotor" :( R.I.P. 2024-2024; a wonderful son, brother, and
    // father.

    // private final VelocityDutyCycle requestTop;
    // private final VelocityDutyCycle requestBottom;
    private final VelocityTorqueCurrentFOC requestTop;
    private final VelocityTorqueCurrentFOC requestBottom;

    private StatusSignal<Double> topMotorStatorCurrent;
    private StatusSignal<Double> topMotorVelocityRPS;
    private StatusSignal<Double> bottomMotorStatorCurrent;
    private StatusSignal<Double> bottomMotorVelocityRPS;

    public ShooterIOFalcon() {
        topMotor = new TalonFX(ShooterConstants.topMotorID); // initializes TalonFX motor 1
        bottomMotor = new TalonFX(ShooterConstants.bottomMotorID); // initializes TalonFX motor 2

        // Top motor configurations
        TalonFXConfiguration topConfigs = new TalonFXConfiguration();
        topMotor.getConfigurator().apply(topConfigs); // reset to default
        topConfigs.MotorOutput.Inverted = ShooterConstants.topMotorInvert;
        topConfigs.MotorOutput.NeutralMode = ShooterConstants.topMotorBrakeMode;
        topConfigs.Slot0.kP = ShooterConstants.kTopP;
        topConfigs.Slot0.kV = ShooterConstants.kTopV;
        topConfigs.Slot0.kS = ShooterConstants.kTopS;
        topConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        topConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.topCurrentLimit;
        topConfigs.TorqueCurrent.PeakForwardTorqueCurrent = ShooterConstants.topCurrentLimit;
        topConfigs.TorqueCurrent.PeakReverseTorqueCurrent = ShooterConstants.topCurrentLimit;
        topMotor.getConfigurator().apply(topConfigs);

        // Bottom motor configurations
        TalonFXConfiguration bottomConfigs = new TalonFXConfiguration();
        bottomMotor.getConfigurator().apply(bottomConfigs);
        bottomConfigs.MotorOutput.Inverted = ShooterConstants.bottomMotorInvert;
        bottomConfigs.MotorOutput.NeutralMode = ShooterConstants.bottomMotorBrakeMode;
        bottomConfigs.Slot0.kP = ShooterConstants.kBottomP;
        bottomConfigs.Slot0.kV = ShooterConstants.kBottomV;
        bottomConfigs.Slot0.kS = ShooterConstants.kBottomS;
        bottomConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        bottomConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.bottomCurrentLimit;
        bottomConfigs.TorqueCurrent.PeakForwardTorqueCurrent = ShooterConstants.bottomCurrentLimit;
        bottomConfigs.TorqueCurrent.PeakReverseTorqueCurrent = ShooterConstants.bottomCurrentLimit;

        bottomMotor.getConfigurator().apply(bottomConfigs);

        // Updates the requests for each motor
        // requestTop = new VelocityDutyCycle(0).withEnableFOC(true);
        // requestBottom = new VelocityDutyCycle(0).withEnableFOC(true);
        requestTop = new VelocityTorqueCurrentFOC(0);
        requestBottom = new VelocityTorqueCurrentFOC(0);

        topMotorStatorCurrent = topMotor.getStatorCurrent();
        topMotorVelocityRPS = topMotor.getVelocity();
        bottomMotorStatorCurrent = bottomMotor.getStatorCurrent();
        bottomMotorVelocityRPS = bottomMotor.getVelocity();
        BaseStatusSignal.setUpdateFrequencyForAll(50, topMotorStatorCurrent, topMotorVelocityRPS, bottomMotorStatorCurrent, bottomMotorVelocityRPS);
        topMotor.optimizeBusUtilization();
        bottomMotor.optimizeBusUtilization();
    }

    /**
     * 
     */
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                topMotorStatorCurrent,
                topMotorVelocityRPS,
                bottomMotorStatorCurrent,
                bottomMotorVelocityRPS);

        inputs.topMotorStatorCurrent = topMotorStatorCurrent.getValueAsDouble();
        inputs.topMotorVelocityRPS = topMotorVelocityRPS.getValueAsDouble();

        inputs.bottomMotorStatorCurrent = bottomMotorStatorCurrent.getValueAsDouble();
        inputs.bottomMotorVelocityRPS = bottomMotorVelocityRPS.getValueAsDouble();
    }

    // /**
    //  * @param rps revs per second
    //  */
    // public void setTopVelocity(double rps) {
    //     topMotor.setControl(requestTop.withVelocity(rps));
    // }

    // /**
    //  * @param rps revs per second
    //  */
    // public void setBottomVelocity(double rps) {
    //     bottomMotor.setControl(requestBottom.withVelocity(rps));
    // }

    /**
     * @param rps revs per second
     */
    public void setTopVelocity(double rps) {
        topMotor.setControl(requestTop.withVelocity(rps));
    }

    /**
     * @param rps revs per second
     */
    public void setBottomVelocity(double rps) {
        bottomMotor.setControl(requestBottom.withVelocity(rps));
    }

    /**
     * 
     * @return returns a double in RPS
     */
    public double getTopVelocity() {
        return topMotor.getVelocity().getValueAsDouble(); // rps
    }

    /**
     * 
     * @return returns a double in RPS
     */
    public double getBottomVelocity() {
        return bottomMotor.getVelocity().getValueAsDouble(); // rps
    }

    public void stopTopMotor() {
        topMotor.stopMotor();
    }

    public void stopBottomMotor() {
        bottomMotor.stopMotor();
    }

    /**
     * testing only
     */
    public void setVelocity(double rpm) {
        double rps = rpm / 60;
        topMotor.setControl(requestTop.withVelocity(rps));
        bottomMotor.setControl(requestBottom.withVelocity(rps));
    }

}