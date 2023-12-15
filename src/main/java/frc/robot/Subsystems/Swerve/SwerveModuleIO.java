package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePosition = 0;
        public double driveVelocity = 0;
        
        public double anglePosition = 0;
        
        public double canCoderPosition = 0;
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {}

    public default void setDrive(ControlModeValue mode, double speed) {}

    public default void setDrive(ControlModeValue mode, double speed, DemandType demandType, double demand) {}

    public default void setDriveNeutralMode(NeutralModeValue mode) {}

    public default void setAngle(ControlModeValue mode, double speed) {}

    public default void setAngle(ControlModeValue mode, double speed, DemandType demandType, double demand) {}

    public default void setAngleNeutralMode(NeutralModeValue mode) {}

    public default void setAngleSelectedSensorPosition(double position) {}
}
