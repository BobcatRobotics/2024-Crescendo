package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveModule[] modules;
    private final SwerveDriveOdometry odometry;

    private final double[] swerveModuleStates = new double[8];
    private final double[] desiredSwerveModuleStates = new double[8];

    public Swerve(GyroIO gyroIO, SwerveModuleIO flIO, SwerveModuleIO frIO, SwerveModuleIO blIO, SwerveModuleIO brIO) {
        this.gyroIO = gyroIO;
        modules = new SwerveModule[] {
            new SwerveModule(flIO, 0),
            new SwerveModule(frIO, 1),
            new SwerveModule(blIO, 2),
            new SwerveModule(brIO, 3)
        };

        odometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getYaw(), getModulePositions());

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdemetry,
            this::getChassisSpeeds,
            this::drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(SwerveConstants.translationKP, SwerveConstants.translationKI, SwerveConstants.translationKD),
                new PIDConstants(SwerveConstants.rotationKP, SwerveConstants.rotationKI, SwerveConstants.rotationKD),
                SwerveConstants.maxSpeed,
                SwerveConstants.driveBaseRadius,
                new ReplanningConfig(true, true)),
            this);
    }

    public void periodic() {
        odometry.update(getYaw(), getModulePositions());  

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Swerve/Gyro", gyroInputs);
        for (SwerveModule module : modules) {
            module.periodic();
        }

        for(SwerveModule mod : modules){
            SmartDashboard.putNumber("Mod " + mod.index + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.index + " Velocity", mod.getState().speedMetersPerSecond);
            desiredSwerveModuleStates[mod.index*2+1] = mod.getDesiredState().speedMetersPerSecond;
            desiredSwerveModuleStates[mod.index*2] = mod.getDesiredState().angle.getRadians();
            swerveModuleStates[mod.index*2+1] = mod.getState().speedMetersPerSecond;
            swerveModuleStates[mod.index*2] = mod.getState().angle.getRadians();
        }

        Logger.recordOutput("Swerve/Rotation", getYaw().getRadians());
        Logger.recordOutput("Swerve/DesiredModuleStates", desiredSwerveModuleStates);
        Logger.recordOutput("Swerve/ModuleStates", swerveModuleStates);
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

        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(desiredSpeeds);
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

    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdemetry(Pose2d pose) {
        odometry.resetPosition(getYaw(), getModulePositions(), pose);
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
