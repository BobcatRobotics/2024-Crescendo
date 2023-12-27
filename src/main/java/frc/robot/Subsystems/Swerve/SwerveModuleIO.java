package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePositionRot = 0.0;
        public double driveVelocityRotPerSec = 0.0;
        public double drivePercentOut = 0.0;
        
        public double anglePositionRot = 0.0;
        
        public double canCoderPositionRot = 0.0;
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {}

    public default void setDriveVelocity(double percent) {}

    public default void stopDrive() {}

    public default void setDriveNeutralMode(NeutralModeValue mode) {}

    public default void setAnglePosition(double position) {}

    public default void stopAngle() {}

    public default void setAngleNeutralMode(NeutralModeValue mode) {}
}