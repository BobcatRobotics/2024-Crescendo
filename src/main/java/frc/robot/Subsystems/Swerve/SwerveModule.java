package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public final int index;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

    private SwerveModuleState desiredState = new SwerveModuleState();

    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/Module" + Integer.toString(index), inputs);
    }

    public SwerveModuleState setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

        io.setAnglePosition(optimizedState.angle.getRotations());

        double velocity = optimizedState.speedMetersPerSecond / SwerveConstants.wheelCircumference;
        io.setDriveVelocity(feedforward.calculate(velocity));

        desiredState = optimizedState;
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
