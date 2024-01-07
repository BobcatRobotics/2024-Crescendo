package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

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

    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;

        lastAngle = getState().angle;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/Module" + Integer.toString(index), inputs);
    }

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

    public void stop() {
        io.stopAngle();
        io.stopDrive();
    }

    public void setAngleNeutralMode(NeutralModeValue mode) {
        io.setAngleNeutralMode(mode);
    }

    public void setDriveNeutralMode(NeutralModeValue mode) {
        io.setDriveNeutralMode(mode);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(inputs.canCoderPositionRot);
    }

    public double getPositionMeters() {
        return inputs.drivePositionRot * SwerveConstants.wheelCircumference;
    }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRotPerSec * SwerveConstants.wheelCircumference;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public double getRawCanCoder() {
        return inputs.rawCanCoderPositionDeg;
    }
}
