package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.SwerveConversions;
import frc.lib.util.ModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public final int index;

    private Rotation2d angleOffset;
    private Rotation2d lastAngle = new Rotation2d();

    private SwerveModuleState desiredState = new SwerveModuleState();

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

    public SwerveModule(SwerveModuleIO io, int index) {
        this.io = io;
        this.index = index;

        angleOffset = io.getModuleOffset();

        resetToAbsolute();
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/SwerveModule" + Integer.toString(index), inputs);
    }

    public void setDriveBrakeMode(boolean brakeMode) {
        io.setDriveNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void setAngleBrakeMode(boolean brakeMode) {
        io.setAngleNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = ModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
        this.desiredState = desiredState;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
            io.setDrive(new DutyCycleOut(percentOutput).withEnableFOC(SwerveConstants.useFOC));
        } else {
            double velocity = SwerveConversions.MPSToRPS(desiredState.speedMetersPerSecond, SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
            io.setDrive(new VelocityDutyCycle(velocity).withEnableFOC(SwerveConstants.useFOC).withFeedForward(feedforward.calculate(desiredState.speedMetersPerSecond)));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d oldAngle = getAngle();
        angle = optimizeTurn(oldAngle, angle);
        io.setAngle(new PositionDutyCycle(SwerveConversions.degreesToRotations(angle.getDegrees(), SwerveConstants.angleGearRatio)).withEnableFOC(SwerveConstants.useFOC).withSlot(0));
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(SwerveConversions.rotationsToDegrees(inputs.anglePosition, SwerveConstants.angleGearRatio));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(inputs.canCoderPosition);
    }

    public double makePositiveDegrees(double anAngle) {
        double degrees = anAngle;
        degrees = degrees % 360;
        if (degrees < 0.0) {
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double makePositiveDegrees(Rotation2d anAngle) {
        return makePositiveDegrees(anAngle.getDegrees());
    }

    public Rotation2d optimizeTurn(Rotation2d oldAngle, Rotation2d newAngle) {
        double steerAngle = makePositiveDegrees(newAngle);
        steerAngle %= (360);
        if (steerAngle < 0.0) {
            steerAngle += 360;
        }

        double difference = steerAngle - oldAngle.getDegrees();
        // Change the target angle so the difference is in the range [-360, 360) instead
        // of [0, 360)
        if (difference >= 360) {
            steerAngle -= 360;
        } else if (difference < -360) {
            steerAngle += 360;
        }
        difference = steerAngle - oldAngle.getDegrees(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can
        // be inverted so the total
        // movement of the module is less than 90 deg
        if (difference > 90 || difference < -90) {
            // Only need to add 180 deg here because the target angle will be put back into
            // the range [0, 2pi)
            steerAngle += 180;
        }

        return Rotation2d.fromDegrees(makePositiveDegrees(steerAngle));
    }

    public void resetToAbsolute() {
        double absolutePosition = SwerveConversions.degreesToRotations(
                makePositiveDegrees(getCanCoder().getDegrees() - angleOffset.getDegrees()),
                SwerveConstants.angleGearRatio);
        io.setAngle(new PositionDutyCycle(absolutePosition).withEnableFOC(SwerveConstants.useFOC).withSlot(0));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                SwerveConversions.RPSToMPS(inputs.driveVelocity, SwerveConstants.wheelCircumference,
                        SwerveConstants.driveGearRatio),
                getAngle());
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                SwerveConversions.rotationsToMeters(inputs.drivePosition, SwerveConstants.wheelCircumference,
                        SwerveConstants.driveGearRatio),
                getAngle());
    }

    public double getDistanceMeters() {
        return SwerveConversions.rotationsToMeters(inputs.drivePosition, SwerveConstants.wheelCircumference,
                SwerveConstants.driveGearRatio);
    }
}
