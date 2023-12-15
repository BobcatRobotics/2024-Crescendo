package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.SwerveConstants;

public class SwerveModuleIOFalcon implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;

    private final CANcoder angleEncoder;

    public SwerveModuleIOFalcon(SwerveModuleConstants moduleConstants) {
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, SwerveConstants.canivore);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.angleMotorID, SwerveConstants.canivore);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID, SwerveConstants.canivore);
        configDriveMotor();
    }

    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePosition = driveMotor.getPosition().getValueAsDouble() * 2048; // Convert to old units
        inputs.driveVelocity = driveMotor.getVelocity().getValueAsDouble() * (2048*10); // Convert to old units

        inputs.anglePosition = angleMotor.getSelectedSensorPosition();

        inputs.canCoderPosition = angleEncoder.getAbsolutePosition();
    }

    /* ----- Drive motor methods ----- */
    private void configDriveMotor(){        
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.setInverted(SwerveConstants.driveMotorInvert);
        driveMotor.setNeutralMode(SwerveConstants.driveNeutralMode);
        driveMotor.setSelectedSensorPosition(0);
    }

    public void setDrive(ControlModeValue mode, double speed) {
        driveMotor.set(mode, speed);
    }

    public void setDrive(ControlModeValue mode, double speed, DemandType demandType, double demand) {
        driveMotor.set(mode, demand, demandType, demand);
    }

    public void setDriveNeutralMode(NeutralModeValue mode) {
        driveMotor.setNeutralMode(mode);
    }

    /* ----- Angle motor methods ----- */
    private void configAngleMotor(){
        angleMotor.configFactoryDefault();
        angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        angleMotor.setInverted(SwerveConstants.angleMotorInvert);
        angleMotor.setNeutralMode(SwerveConstants.angleNeutralMode);
    }

    public void setAngle(ControlModeValue mode, double speed) {
        angleMotor.set(mode, speed);
    }

    public void setAngle(ControlModeValue mode, double speed, DemandType demandType, double demand) {
        angleMotor.set(mode, demand, demandType, demand);
    }

    public void setAngleNeutralMode(NeutralModeValue mode) {
        angleMotor.setNeutralMode(mode);
    }

    public void setAngleSelectedSensorPosition(double position) {
        angleMotor.setSelectedSensorPosition(position);
    }

    /* ----- Angle encoder methods ----- */
    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }
}
