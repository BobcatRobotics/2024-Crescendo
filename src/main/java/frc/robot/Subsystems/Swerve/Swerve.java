package frc.robot.Subsystems.Swerve;

import java.util.Optional;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.limelightConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Util.BobcatUtil;

public class Swerve extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveModule[] modules;
    private final Vision shooterLeftVision;
    private final Vision shooterRightVision;
    private final Vision IntakeVision;
    private final SwerveDrivePoseEstimator poseEstimator;
    // private final SwerveDriveOdometry odometry;

    private final double[] swerveModuleStates = new double[8];
    private final double[] desiredSwerveModuleStates = new double[8];

    private SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    private Rotation2d ppRotationOverride;

    // private SwerveSetpointGenerator setpointGenerator;
    // private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
    // new ChassisSpeeds(),
    // new SwerveModuleState[] {
    // new SwerveModuleState(),
    // new SwerveModuleState(),
    // new SwerveModuleState(),
    // new SwerveModuleState()
    // });

    private final PIDController rotationPID;
    private double lastMovingYaw = 0.0;
    private boolean rotating = false;

    static final Lock odometryLock = new ReentrantLock();

    private final LoggedDashboardNumber driveToPoseX = new LoggedDashboardNumber("desired x");
    private final LoggedDashboardNumber driveToPoseY = new LoggedDashboardNumber("desired y");
    Pose2d desiredPose = new Pose2d();
    private Rotation2d lastYaw = new Rotation2d();

    public Swerve(GyroIO gyroIO, SwerveModuleIO flIO, SwerveModuleIO frIO, SwerveModuleIO blIO, SwerveModuleIO brIO,
            Vision intakeVision, Vision shooterLeftVision, Vision shooterRightVision) {
        this.IntakeVision = intakeVision;
        this.shooterLeftVision = shooterLeftVision;
        this.shooterRightVision = shooterRightVision;
        this.gyroIO = gyroIO;
        modules = new SwerveModule[] {
                new SwerveModule(flIO, 0),
                new SwerveModule(frIO, 1),
                new SwerveModule(blIO, 2),
                new SwerveModule(brIO, 3)
        };

        PhoenixOdometryThread.getInstance().start();

        rotationPID = new PIDController(SwerveConstants.teleopRotationKP, SwerveConstants.teleopRotationKI,
                SwerveConstants.teleopRotationKD);

        // Using last year's default deviations, need to tune
        
        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics, getYaw(), getModulePositions(),
                new Pose2d(), SwerveConstants.autostateStdDevs, VecBuilder.fill(0, 0, 0));

        // setpointGenerator =
        // SwerveSetpointGenerator.builder()
        // .kinematics(SwerveConstants.swerveKinematics)
        // .moduleLocations(SwerveConstants.moduleTranslations)
        // .build();

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose,
                this::getChassisSpeeds,
                this::drive,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(SwerveConstants.translationKP, SwerveConstants.translationKI,
                                SwerveConstants.translationKD),
                        new PIDConstants(SwerveConstants.rotationKP, SwerveConstants.rotationKI,
                                SwerveConstants.rotationKD),
                        SwerveConstants.maxSpeed,
                        SwerveConstants.driveBaseRadius,
                        new ReplanningConfig(true, true)),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    return BobcatUtil.getAlliance() == DriverStation.Alliance.Red;

                },
                this);
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

    }

    public Optional<Rotation2d> getRotationTargetOverride() {
        if (getRotationTarget() != null) {
            return Optional.of(getRotationTarget());
        } else {
            return Optional.empty();
        }
    }

    public Rotation2d getRotationTarget() {
        return ppRotationOverride; // the rotation2d this returns will override the one in pathplanner, if
                                   // null, the default pathplanner rotation will be used
    }

    public void setRotationTarget(Rotation2d target) {
        ppRotationOverride = target;
    }

    public void periodic() {
        if (BobcatUtil.getAlliance() == Alliance.Blue) {
            shooterLeftVision.setPriorityID(LimelightConstants.blueSpeakerTag, LimelightConstants.shooterLeft.name);
            shooterRightVision.setPriorityID(LimelightConstants.blueSpeakerTag, LimelightConstants.shooterRight.name);
        } else {
            shooterLeftVision.setPriorityID(LimelightConstants.redSpeakerTag, LimelightConstants.shooterLeft.name);
            shooterRightVision.setPriorityID(LimelightConstants.redSpeakerTag, LimelightConstants.shooterRight.name);
        }
        odometryLock.lock();
        gyroIO.updateInputs(gyroInputs);
        for (SwerveModule module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        Logger.recordOutput("Swerve/YawSetpoint", lastMovingYaw);
        Logger.recordOutput("Swerve/CurrentYaw", getYaw().getRadians());
        Logger.processInputs("Swerve/Gyro", gyroInputs);

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
            }

            if (gyroInputs.connected) { // Use gyro when connected
                Rotation2d yaw = getYaw();
                lastYaw = yaw;
            } else { // If disconnected or sim, use angular velocity
                Rotation2d yaw = lastYaw.plus(
                        Rotation2d.fromRadians(getChassisSpeeds().omegaRadiansPerSecond * Constants.loopPeriodSecs));
                lastYaw = yaw;
            }

            poseEstimator.updateWithTime(sampleTimestamps[i], lastYaw, modulePositions);
            swerveModulePositions = modulePositions;
        }

        for (SwerveModule mod : modules) {
            SmartDashboard.putNumber("Mod " + mod.index + " Angle", mod.getRawCanCoder());
            desiredSwerveModuleStates[mod.index * 2 + 1] = mod.getDesiredState().speedMetersPerSecond;
            desiredSwerveModuleStates[mod.index * 2] = mod.getDesiredState().angle.getDegrees();
            swerveModuleStates[mod.index * 2 + 1] = mod.getState().speedMetersPerSecond;
            swerveModuleStates[mod.index * 2] = mod.getState().angle.getDegrees();
        }

        Logger.recordOutput("Swerve/Rotation", gyroInputs.yawPosition.getDegrees());
        Logger.recordOutput("Swerve/DesiredModuleStates", desiredSwerveModuleStates);
        Logger.recordOutput("Swerve/ModuleStates", swerveModuleStates);
        Logger.recordOutput("Swerve/Pose", getPose());
        if (DriverStation.isDisabled()) {
            for (SwerveModule mod : modules) {
                mod.stop();
            }
        }
        desiredPose = new Pose2d(driveToPoseX.get(), driveToPoseY.get(), new Rotation2d());
        Logger.recordOutput("Swerve/DesiredPose", desiredPose);

        // Update PoseEstimator if at least 1 tag is in view
        if (Math.abs(shooterRightVision.getBotPose().getY()) <= 50
                && Math.abs(shooterRightVision.getBotPose().getY()) != 0) {
            // standard deviations are (distance to nearest apriltag)/2 for x and y and 10
            // degrees for theta
            poseEstimator.addVisionMeasurement((shooterRightVision.getBotPose()),
                    (shooterRightVision.getPoseTimestamp()),
                    VecBuilder.fill(shooterRightVision.getDistToTag() / getStdDev(),
                            shooterRightVision.getDistToTag() / getStdDev(), Units.degreesToRadians(60)));
            Logger.recordOutput("shooterrightvisiondist", shooterRightVision.getDistToTag());
        }
        if (Math.abs(shooterLeftVision.getBotPose().getY()) <= 50
                && Math.abs(shooterLeftVision.getBotPose().getY()) != 0) {
            poseEstimator.addVisionMeasurement((shooterLeftVision.getBotPose()), (shooterLeftVision.getPoseTimestamp()),
                    VecBuilder.fill(shooterLeftVision.getDistToTag() / getStdDev(),
                            shooterLeftVision.getDistToTag() / getStdDev(),
                            Units.degreesToRadians(60)));
            Logger.recordOutput("shooterleftvisiondist", shooterLeftVision.getDistToTag());

        }

        SmartDashboard.putNumber("distance to speaker", getDistanceToSpeaker());

    }

    public double getStdDev(){
        return DriverStation.isAutonomous() ? LimelightConstants.autostdDev : LimelightConstants.telestdDev;
    }

    public Pose2d getPathfindingPose() {

        return desiredPose;
    }

    /**
     * Gets the current yaw of the gyro or the estimated yaw if the gyro is
     * disconnected
     * 
     * @return current yaw of the gyro
     */
    public Rotation2d getYaw() {
        if (gyroInputs.connected) { // Use gyro when connected
            return gyroInputs.yawPosition;
        } else { // If disconnected or sim, use angular velocity
            return lastYaw;
        }
    }

    /**
     * Makes the swerve drive move
     * 
     * @param translation   desired x and y speeds of the swerve drive in meters per
     *                      second
     * @param rotation      desired rotation speed of the swerve drive in radians
     *                      per second
     * @param fieldRelative whether the values should be field relative or not
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean snapToAmp,
            boolean snapToSpeaker, double angleToSpeaker) {

        ChassisSpeeds desiredSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getYaw())
                : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation);

        if (snapToSpeaker) {
            desiredSpeeds.omegaRadiansPerSecond = rotationPID.calculate(get0to2Pi(getYaw().getRadians()),
                    get0to2Pi(angleToSpeaker));
            lastMovingYaw = getYaw().getRadians();
        } else if (snapToAmp) {
            desiredSpeeds.omegaRadiansPerSecond = rotationPID.calculate(get0to2Pi(getYaw().getRadians()),
                    Math.PI / 2);
            lastMovingYaw = getYaw().getRadians();
        } else {
            if (rotation == 0) {
                if (rotating) {
                    rotating = false;
                    lastMovingYaw = getYaw().getRadians();
                }
                desiredSpeeds.omegaRadiansPerSecond = rotationPID.calculate(get0to2Pi(getYaw().getRadians()),
                        get0to2Pi(lastMovingYaw));
            } else {
                rotating = true;
            }
        }

        desiredSpeeds = ChassisSpeeds.discretize(desiredSpeeds, Constants.loopPeriodSecs);

        // currentSetpoint =
        // setpointGenerator.generateSetpoint(SwerveConstants.moduleLimits,
        // currentSetpoint, desiredSpeeds, Constants.loopPeriodSecs);

        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(desiredSpeeds);
        // SwerveModuleState[] swerveModuleStates = currentSetpoint.moduleStates();
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : modules) {
            mod.setDesiredState(swerveModuleStates[mod.index]);
        }
    }

    /**
     * 
     * Make the swerve drive move
     * 
     * @param targetSpeeds the desired chassis speeds
     */
    public void drive(ChassisSpeeds targetSpeeds) {
        targetSpeeds = ChassisSpeeds.discretize(targetSpeeds, Constants.loopPeriodSecs);

        lastMovingYaw = getYaw().getRadians();

        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : modules) {
            mod.setDesiredState(swerveModuleStates[mod.index]);
        }
    }

    /**
     * Sets all of the modules to their desired states
     * 
     * @param desiredStates array of states for the modules to be set to
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : modules) {
            mod.setDesiredState(desiredStates[mod.index]);
        }
    }

    /**
     * Gets distance to nearest apriltag
     * 
     * @return distance to nearest apriltag in meters
     */

    // public double getDistance(String limelight) {
    // // return
    // PoseEstimator.getEstimatedPosition().getTranslation().getDistance(new
    // Pose2d(LimelightHelpers.getTargetPose3d_RobotSpace(limelight).getX(),
    // LimelightHelpers.getTargetPose3d_RobotSpace(limelight).getY()).getTranslation());
    // //getting x distance to target
    // return LimelightHelpers.getTargetPose_RobotSpace(limelight)[0];
    // }

    /**
     * Gets all of the current module states
     * 
     * @return array of the current module states
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : modules) {
            states[mod.index] = mod.getState();
        }
        return states;
    }

    /**
     * Gets all of the current module positions
     * 
     * @return array of the current module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : modules) {
            positions[mod.index] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Gets ths current chassis speeds
     * 
     * @return current chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Gets the current pose, according to our odometry
     * 
     * @return current pose in meters
     */
    public Pose2d getPose() {
        // return odometry.getPoseMeters();
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets our odometry to desired pose
     * 
     * @param pose pose to set odometry to
     */
    public void resetPose(Pose2d pose) {
        // odometry.resetPosition(getYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /**
     * Sets the current gyro yaw to 0 degrees
     */
    public void zeroGyro() {
        gyroIO.setYaw(0);
        lastMovingYaw = 0;
        lastYaw = Rotation2d.fromDegrees(0);
    }

    /**
     * Sets the current gyro yaw to 180 degrees
     */
    public void reverseZeroGyro() {
        gyroIO.setYaw(180);
        lastMovingYaw = 180;
        lastYaw = Rotation2d.fromDegrees(180);
    }

    /**
     * Sets the modules to an X shape to make the robot harder to push
     */
    public void configToX() {
        modules[0].setDesiredState(new SwerveModuleState(1, new Rotation2d(Math.toRadians(45))));
        modules[1].setDesiredState(new SwerveModuleState(1, new Rotation2d(Math.toRadians(315))));
        modules[2].setDesiredState(new SwerveModuleState(1, new Rotation2d(Math.toRadians(315))));
        modules[3].setDesiredState(new SwerveModuleState(1, new Rotation2d(Math.toRadians(45))));
    }

    /**
     * Stops the swerve drive
     */
    public void stop() {
        drive(new ChassisSpeeds());
    }

    public Command driveToPose(Pose2d pose) {

        return AutoBuilder.pathfindToPose(pose,
                new PathConstraints(Constants.SwerveConstants.maxSpeed, Constants.SwerveConstants.maxAccel,
                        Constants.SwerveConstants.maxAngularVelocity,
                        Constants.SwerveConstants.maxAngularAcceleration));
    }

    /**
     * 
     * @return DISTANCE TO SPEAKER OF CURRENT ALLIANCE IN METERS :D
     */
    public double getDistanceToSpeaker() {
        return BobcatUtil.getAlliance() == Alliance.Blue
                ? getPose().getTranslation().getDistance(FieldConstants.blueSpeakerPose)
                : getPose().getTranslation().getDistance(FieldConstants.redSpeakerPose);
    }

    public Translation2d getTranslationToSpeaker() {
        return BobcatUtil.getAlliance() == Alliance.Blue
                ? FieldConstants.blueSpeakerPose.minus(getPose().getTranslation())
                : FieldConstants.redSpeakerPose.minus(getPose().getTranslation());
    }

    /**
     * 
     * 
     * r^2 = 0.983
     */
    public double calcAngleBasedOnRealRegression() {
        double distance = getDistanceToSpeaker();
        Logger.recordOutput("Spivit/DesiredAngle", 291 * Math.pow(distance, -0.074));
        return 291 * Math.pow(distance, -0.074);

    }

    /**
     * 
     * 
     * r^2 = 0.996
     */
    public double calcAngleBasedOnEstimatorRegression() {
        double distance = getDistanceToSpeaker();
        return 291 * Math.pow(distance, -0.0762);
    }

    public double get0to2Pi(double rad) {
        rad = rad % (2 * Math.PI);
        if (rad < (2 * Math.PI))
            rad += (2 * Math.PI);
        return rad;
    }

    public boolean aligned() {
        return Math.abs(Math.toDegrees(rotationPID.getPositionError())) <= SwerveConstants.rotationToleranceAlignment;
    }

    /**
     * 
     * @return the angle to the speaker in radians
     * @deprecated
     */
    public double getAngleToSpeaker() {
        Translation2d speaker = getTranslationToSpeaker();
        return Math.atan(speaker.getY() / speaker.getX());
    }

    public Rotation2d getAngleToSpeakerApriltag() {

        //if (shooterLeftVision.getTV() && shooterRightVision.getTV()) {
            return Rotation2d.fromRadians(get0to2Pi(getYaw().getRadians()
                    - (-((shooterLeftVision.getTX().getRadians()) + shooterRightVision.getTX().getRadians()) / 2)));
        //}
        // else if(shooterLeftVision.getTV()){
        // return shooterLeftVision.getTX().unaryMinus();
        // }else if(shooterRightVision.getTV()){
        // return shooterRightVision.getTX().unaryMinus();
        // }
        //else {
          //  return Rotation2d.fromRadians(getAngleToSpeaker());
        //}
    }

}