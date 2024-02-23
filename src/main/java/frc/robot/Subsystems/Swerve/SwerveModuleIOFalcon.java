package frc.robot.Subsystems.Swerve;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.ModuleConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleIOFalcon implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder angleEncoder;

    private final Rotation2d encoderOffset;

    private final DutyCycleOut driveRequest;
    private final DutyCycleOut angleRequest;
    private final PositionDutyCycle sysidAngle; //sysid only
    private final VoltageOut driveCharacterizationControl = new VoltageOut(0); //also sysid only


    private final Queue<Double> timestampQueue;

    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;

    private final Queue<Double> anglePositionQueue;
    private final StatusSignal<Double> angleAbsolutePosition;
    private final StatusSignal<Double> driveAppliedVolts;


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
        sysidAngle = new PositionDutyCycle(0).withEnableFOC(true);

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        drivePosition = driveMotor.getPosition();
        drivePositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(driveMotor, driveMotor.getPosition());
        driveVelocity = driveMotor.getVelocity();

        driveAppliedVolts = driveMotor.getMotorVoltage();
        angleAbsolutePosition = angleEncoder.getAbsolutePosition();
        anglePositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(angleEncoder, angleEncoder.getPosition());

        BaseStatusSignal.setUpdateFrequencyForAll(
            250.0, drivePosition, driveVelocity, angleAbsolutePosition, driveAppliedVolts);
        driveMotor.optimizeBusUtilization();
        angleMotor.optimizeBusUtilization();
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.offset = encoderOffset;
        BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        angleAbsolutePosition,
        driveAppliedVolts);

        inputs.drivePositionRot = drivePosition.getValueAsDouble() / SwerveConstants.driveGearRatio;
        inputs.driveVelocityRotPerSec = driveVelocity.getValueAsDouble() / SwerveConstants.driveGearRatio;
        inputs.appliedDriveVoltage = driveAppliedVolts.getValueAsDouble();
        inputs.canCoderPositionRot = Rotation2d.fromRadians(MathUtil.angleModulus(Rotation2d.fromRotations(angleAbsolutePosition.getValueAsDouble()).minus(encoderOffset).getRadians())).getRotations();
        inputs.rawCanCoderPositionDeg = Rotation2d.fromRotations(angleAbsolutePosition.getValueAsDouble()).getDegrees(); // Used only for shuffleboard to display values to get offsets

        inputs.odometryTimestamps =
            timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
            drivePositionQueue.stream()
                .mapToDouble((Double value) -> Units.rotationsToRadians(value) / SwerveConstants.driveGearRatio)
                .toArray();
        inputs.odometryAnglePositions =
            anglePositionQueue.stream()
                // .map((Double value) -> Rotation2d.fromRadians(MathUtil.angleModulus(Rotation2d.fromRotations(value).minus(encoderOffset).getRadians())))
                .map((Double value) -> Rotation2d.fromRotations(value))
                .toArray(Rotation2d[]::new);

        timestampQueue.clear();
        drivePositionQueue.clear();
        anglePositionQueue.clear();
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


    @Override
    public void setAngle(Rotation2d angle){
        angleMotor.setControl(sysidAngle.withPosition(angle.getRotations()));
    }

    @Override
    public void runDriveCharacterization(double volts) {
        driveMotor.setControl(driveCharacterizationControl.withOutput(volts));
    }

}