package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.ModuleConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleIOFalcon implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder angleEncoder;

    private final Rotation2d encoderOffset;

    private final DutyCycleOut driveRequest;
    private final DutyCycleOut angleRequest;

    public SwerveModuleIOFalcon(ModuleConstants moduleConstants) {
        encoderOffset = moduleConstants.angleOffset;

        angleEncoder = new CANcoder(moduleConstants.cancoderID, SwerveConstants.canivore);
        configAngleEncoder();
        angleMotor = new TalonFX(moduleConstants.angleMotorID, SwerveConstants.canivore);
        configAngleMotor();
        driveMotor = new TalonFX(moduleConstants.driveMotorID, SwerveConstants.canivore);
        configDriveMotor();

        driveRequest = new DutyCycleOut(0.0).withEnableFOC(SwerveConstants.useFOC);
        angleRequest = new DutyCycleOut(0.0).withEnableFOC(SwerveConstants.useFOC);
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionRot = driveMotor.getPosition().getValueAsDouble() / SwerveConstants.driveGearRatio;
        inputs.driveVelocityRotPerSec = driveMotor.getVelocity().getValueAsDouble() / SwerveConstants.driveGearRatio;
        inputs.drivePercentOut = driveMotor.getDutyCycle().getValueAsDouble();

        inputs.anglePercentOut = angleMotor.getDutyCycle().getValueAsDouble();

        inputs.canCoderPositionRot = Rotation2d.fromRadians(MathUtil.angleModulus(Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble()).minus(encoderOffset).getRadians())).getRotations();
        inputs.rawCanCoderPositionDeg = Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble()).getDegrees(); // Used only for shuffleboard to display values to get offsets
    }

    /**
     * Sets the percent out of the drive motor
     * @param percent percent to set it to, from -1.0 to 1.0
     */
    public void setDrivePercentOut(double percent) {
        driveMotor.setControl(driveRequest.withOutput(percent));
    }

    /**
     * Stops the drive motor
     */
    public void stopDrive() {
        driveMotor.stopMotor();
    }

    /**
     * Sets the neutral mode of the drive motor
     * @param mode mode to set it to
     */
    public void setDriveNeutralMode(NeutralModeValue mode) {
        driveMotor.setNeutralMode(mode);
    }

    /**
     * Sets the percent out of the angle motor
     * @param percent percent to set it to, from -1.0 to 1.0
     */
    public void setAnglePercentOut(double percent) {
        angleMotor.setControl(angleRequest.withOutput(percent));
    }

    /**
     * Stops the angle motor
     */
    public void stopAngle() {
        angleMotor.stopMotor();
    }

    /**
     * Sets the neutral mode of the angle motor
     * @param mode mode to set it to
     */
    public void setAngleNeutralMode(NeutralModeValue mode) {
        angleMotor.setNeutralMode(mode);
    }

    /**
     * Applies all configurations to the drive motor
     */
    public void configDriveMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());

        config.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.driveSupplyCurrentLimitEnable;
        config.CurrentLimits.SupplyCurrentLimit = SwerveConstants.driveSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.driveSupplyCurrentThreshold;
        config.CurrentLimits.SupplyTimeThreshold = SwerveConstants.driveSupplyTimeThreshold;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        config.OpenLoopRamps.TorqueOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;

        config.MotorOutput.Inverted = SwerveConstants.driveMotorInvert;
        config.MotorOutput.NeutralMode = SwerveConstants.driveNeutralMode;

        driveMotor.getConfigurator().apply(config);

        driveMotor.getConfigurator().setPosition(0);
    }

    /**
     * Applies all configurations to the angle motor
     */
    public void configAngleMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        angleMotor.getConfigurator().apply(new TalonFXConfiguration());

        config.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.angleSupplyCurrentLimitEnable;
        config.CurrentLimits.SupplyCurrentLimit = SwerveConstants.angleSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.angleSupplyCurrentThreshold;
        config.CurrentLimits.SupplyTimeThreshold = SwerveConstants.angleSupplyTimeThreshold;

        config.MotorOutput.Inverted = SwerveConstants.angleMotorInvert;
        config.MotorOutput.NeutralMode = SwerveConstants.angleNeutralMode;

        angleMotor.getConfigurator().apply(config);
    }


    /**
     * Applies all configurations to the angle absolute encoder
     */
    public void configAngleEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());

        config.MagnetSensor.AbsoluteSensorRange = SwerveConstants.sensorRange;
        config.MagnetSensor.SensorDirection = SwerveConstants.sensorDirection;

        angleEncoder.getConfigurator().apply(config);
    }
}