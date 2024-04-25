package frc.lib.util.BobcatLib.Swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Swerve.SwerveModuleIOInputsAutoLogged;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public final int index;

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS,
            SwerveConstants.driveKV, SwerveConstants.driveKA);
    private PIDController driveController = new PIDController(SwerveConstants.driveKP, SwerveConstants.driveKI,
            SwerveConstants.driveKD);
    private PIDController angleController = new PIDController(SwerveConstants.angleKP, SwerveConstants.angleKI,
            SwerveConstants.angleKD);

    private SwerveModuleState desiredState = new SwerveModuleState();

    private Rotation2d lastAngle;

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;

        angleController.enableContinuousInput(0, 2 * Math.PI);

        lastAngle = getState().angle;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/Module" + Integer.toString(index), inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * (SwerveConstants.wheelCircumference / (2 * Math.PI));
            Rotation2d angle =
                inputs.odometryAnglePositions[i].minus(
                    inputs.offset != null ? inputs.offset : new Rotation2d());
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    /**
     * Sets the swerve module to the desired state
     * @param state the desired state of the swerve module
     * @return the optimized swerve module state that it was set to
     */
    public SwerveModuleState setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : optimizedState.angle;

        // It is important that we use radians for the PID
        // so we can update the drive speed as shown below
        double output = MathUtil.clamp(
                angleController.calculate(getAngle().getRadians(), optimizedState.angle.getRadians()), -1.0, 1.0);
        io.setAnglePercentOut(output);

        // Update velocity based on turn error
        optimizedState.speedMetersPerSecond *= Math.cos(angleController.getPositionError());

        double velocity = optimizedState.speedMetersPerSecond / SwerveConstants.wheelCircumference;
        double velocityOut = MathUtil.clamp(
                driveController.calculate(inputs.driveVelocityRotPerSec, velocity)
                        + driveFeedforward.calculate(velocity),
                -1.0, 1.0);
        if (velocity == 0) {
            velocityOut = 0;
        }
        io.setDrivePercentOut(velocityOut);

        desiredState = optimizedState;
        lastAngle = angle;
        return optimizedState;
    }

    /**
     * Stops the drive and angle motors
     */
    public void stop() {
        io.stopAngle();
        io.stopDrive();
    }

    /**
     * Sets the neutral mode of the angle motor
     * @param mode the mode to set it to
     */
    public void setAngleNeutralMode(NeutralModeValue mode) {
        io.setAngleNeutralMode(mode);
    }

    /**
     * Sets the neutral mode of the drive motor
     * @param mode the mode to set it to
     */
    public void setDriveNeutralMode(NeutralModeValue mode) {
        io.setDriveNeutralMode(mode);
    }

    /**
     * Gets the current angle of the swerve module from the CANcoder
     * @return the angle of the module
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(inputs.canCoderPositionRot);
    }

    /**
     * Gets the current position of the drive motor in meters
     * @return drive motor position, in meters
     */
    public double getPositionMeters() {
        return inputs.drivePositionRot * SwerveConstants.wheelCircumference;
    }

    /**
     * Gets the current velocity of the drive motor in meters per second
     * @return velocity, in meter per second
     */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRotPerSec * SwerveConstants.wheelCircumference;
    }

    /**
     * Gets the current position of the swerve module
     * @return the swerve module position
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /**
     * Gets the current state of the swerve module
     * @return the swerve module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /**
     * Gets the current desired state that the swerve module has been set to
     * @return the desired state
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Gets the raw value of the CANcoder, before the offset is applied. Used only for SmartDashboard
     * @return CANcoder position, in degrees
     */
    public double getRawCanCoder() {
        return inputs.rawCanCoderPositionDeg;
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    public double getDriveAcceleration(){
        return inputs.driveAcceleration;
    }
}
