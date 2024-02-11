package frc.robot.Subsystems.Swerve;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.ModuleConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleIOFalcon implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder angleEncoder;

    private final Rotation2d encoderOffset;

    private final DutyCycleOut driveRequest;
    private final DutyCycleOut angleRequest;

    private final Queue<Double> timestampQueue;

    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> drivePercentOut;

    private final Queue<Double> anglePositionQueue;
    private final StatusSignal<Double> angleAbsolutePosition;
    private final StatusSignal<Double> anglePercentOut;

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

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        drivePosition = driveMotor.getPosition();
        drivePositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(driveMotor, driveMotor.getPosition());
        driveVelocity = driveMotor.getVelocity();
        drivePercentOut = driveMotor.getDutyCycle();

        angleAbsolutePosition = angleEncoder.getAbsolutePosition();
        anglePositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(angleEncoder, angleEncoder.getPosition());
        anglePercentOut = angleMotor.getDutyCycle();

        BaseStatusSignal.setUpdateFrequencyForAll(
            250.0, drivePosition, driveVelocity, drivePercentOut, angleAbsolutePosition, anglePercentOut);
        driveMotor.optimizeBusUtilization();
        angleMotor.optimizeBusUtilization();
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.offset = encoderOffset;
        BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        drivePercentOut,
        angleAbsolutePosition,
        anglePercentOut);

        inputs.drivePositionRot = drivePosition.getValueAsDouble() / SwerveConstants.driveGearRatio;
        inputs.driveVelocityRotPerSec = driveVelocity.getValueAsDouble() / SwerveConstants.driveGearRatio;
        inputs.drivePercentOut = drivePercentOut.getValueAsDouble();

        inputs.anglePercentOut = anglePercentOut.getValueAsDouble();

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

        double totalTime = 0;
        for (int i = 1; i < inputs.odometryTimestamps.length; i++) {
            totalTime += (inputs.odometryTimestamps[i]-inputs.odometryTimestamps[i-1]);
        }
        double avgTime = totalTime / (inputs.odometryTimestamps.length-1);
        inputs.freq = 1/avgTime;

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
}