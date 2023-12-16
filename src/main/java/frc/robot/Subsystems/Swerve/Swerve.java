package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveModule[] modules;

    public Swerve(GyroIO gyroIO, SwerveModuleIO flIO, SwerveModuleIO frIO, SwerveModuleIO blIO, SwerveModuleIO brIO) {
        this.gyroIO = gyroIO;
        modules = new SwerveModule[] {
            new SwerveModule(flIO, 0),
            new SwerveModule(frIO, 1),
            new SwerveModule(blIO, 2),
            new SwerveModule(brIO, 3)
        };

        Timer.delay(1.0);

        resetModulesToAbsolute();
    }

    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Swerve/Gyro", gyroInputs);
        for (SwerveModule module : modules) {
            module.periodic();
        }

        if (DriverStation.isDisabled()) {
            resetModulesToAbsolute();
        }

        for(SwerveModule mod : modules){
            SmartDashboard.putNumber("Mod " + mod.index + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.index + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyroInputs.yawPositionDeg);
    }

    public double getPitch() {
        return gyroInputs.pitchPositionDeg;
    }

    public double getRoll() {
        return gyroInputs.rollPositionDeg;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds desiredSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation, 
            getYaw()
        )
        : new ChassisSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation
        );

        SwerveModuleState[] swerveModuleStates =
        SwerveConstants.swerveKinematics.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for(SwerveModule mod : modules){
            mod.setDesiredState(swerveModuleStates[mod.index], isOpenLoop);
        }
    }

    public void drive(ChassisSpeeds targetSpeeds) {
        SwerveModuleState[] swerveModuleStates =
            SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for(SwerveModule mod : modules){
            mod.setDesiredState(swerveModuleStates[mod.index], true);
        }
    }

    public void enableBrakeMode(boolean enable) {
        for (SwerveModule mod : modules) {
            mod.setDriveBrakeMode(enable);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);
        
        for(SwerveModule mod : modules){
            mod.setDesiredState(desiredStates[mod.index], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : modules){
            states[mod.index] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : modules){
            positions[mod.index] = mod.getPosition();
        }
        return positions;
    }

    public void resetModulesToAbsolute() {
        for(SwerveModule mod : modules){
            mod.resetToAbsolute();
        }
    }

    public void zeroGyro(){
        gyroIO.setYaw(0);
    }

    public void reverseZeroGyro() {
        gyroIO.setYaw(180);
    }

    public void configToX(){
        modules[0].setDesiredState(new SwerveModuleState(1, new Rotation2d(Math.toRadians(45))), true);
        modules[1].setDesiredState(new SwerveModuleState(1, new Rotation2d(Math.toRadians(315))), true);
        modules[2].setDesiredState(new SwerveModuleState(1, new Rotation2d(Math.toRadians(315))), true);
        modules[3].setDesiredState(new SwerveModuleState(1, new Rotation2d(Math.toRadians(45))), true);
    }
}
