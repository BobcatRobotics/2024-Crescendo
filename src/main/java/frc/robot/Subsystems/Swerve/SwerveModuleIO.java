package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePosition = 0;
        public double driveVelocity = 0;
        
        public double anglePosition = 0;
        
        public double canCoderPosition = 0;
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {}

    public default void setDrive(ControlRequest request) {}

    public default void setDriveNeutralMode(NeutralModeValue mode) {}

    public default void setAngle(ControlRequest request) {}

    public default void setAngleNeutralMode(NeutralModeValue mode) {}

    public default Rotation2d getModuleOffset() {return null;}
}
