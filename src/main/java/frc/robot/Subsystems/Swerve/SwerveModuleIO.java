package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public Rotation2d offset = new Rotation2d();

        public double drivePositionRot = 0.0;
        public double driveVelocityRotPerSec = 0.0;
        public double drivePercentOut = 0.0;
        
        public double anglePercentOut = 0.0;
        
        public double canCoderPositionRot = 0.0;
        public double rawCanCoderPositionDeg = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryAnglePositions = new Rotation2d[] {};
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {}

    /**
     * Sets the percent out of the drive motor
     * @param percent percent to set it to, from -1.0 to 1.0
     */
    public default void setDrivePercentOut(double percent) {}

    /**
     * Stops the drive motor
     */
    public default void stopDrive() {}

    /**
     * Sets the neutral mode of the drive motor
     * @param mode mode to set it to
     */
    public default void setDriveNeutralMode(NeutralModeValue mode) {}

    /**
     * Sets the percent out of the angle motor
     * @param percent percent to set it to, from -1.0 to 1.0
     */
    public default void setAnglePercentOut(double percent) {}

    /**
     * Stops the angle motor
     */
    public default void stopAngle() {}

    /**
     * Sets the neutral mode of the angle motor
     * @param mode mode to set it to
     */
    public default void setAngleNeutralMode(NeutralModeValue mode) {}
}
