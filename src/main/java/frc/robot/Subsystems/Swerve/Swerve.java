package frc.robot.Subsystems.Swerve;

import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Quartic;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Util.BobcatUtil;
import frc.robot.Util.RobotPoseLookup;

public class Swerve extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveModule[] modules;
    private final Vision shooterLeftVision;
    private final Vision shooterRightVision;
    private final Vision IntakeVision;
    //private final Vision shooterCenterVision;
    //private final Vision IntakeTagVision;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d = new Field2d();

    // private final SwerveDriveOdometry odometry;

    private final double[] swerveModuleStates = new double[8];
    private final double[] desiredSwerveModuleStates = new double[8];

    private SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    private Rotation2d ppRotationOverride;
    private final RobotPoseLookup poseLookup;
    public boolean isSmoothieAligned = false;

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
    private final PIDController autoAlignPID;
    private final PIDController passPID;
    private double lastMovingYaw = 0.0;
    private boolean rotating = false;
    Timer timer = new Timer();
    static final Lock odometryLock = new ReentrantLock();

    Pose2d desiredPose = new Pose2d();
    private Rotation2d lastYaw = new Rotation2d();

    public Swerve(GyroIO gyroIO, SwerveModuleIO flIO, SwerveModuleIO frIO, SwerveModuleIO blIO, SwerveModuleIO brIO,
            Vision intakeVision, Vision shooterLeftVision, Vision shooterRightVision, Vision shooterCenterVision,
            Vision intakeTagVision) {
        this.IntakeVision = intakeVision;
        this.shooterLeftVision = shooterLeftVision;
        this.shooterRightVision = shooterRightVision;
        //this.shooterCenterVision = shooterCenterVision;
        //this.IntakeTagVision = intakeTagVision;
        this.gyroIO = gyroIO;
        SmartDashboard.putData(field2d);
        modules = new SwerveModule[] {
                new SwerveModule(flIO, 0),
                new SwerveModule(frIO, 1),
                new SwerveModule(blIO, 2),
                new SwerveModule(brIO, 3)
        };

        PhoenixOdometryThread.getInstance().start();

        rotationPID = new PIDController(SwerveConstants.teleopRotationKP, SwerveConstants.teleopRotationKI,
                SwerveConstants.teleopRotationKD);
        rotationPID.enableContinuousInput(0, 2 * Math.PI);
        autoAlignPID = new PIDController(SwerveConstants.autoAlignRotationKP, SwerveConstants.autoAlignRotationKI,
                SwerveConstants.autoAlignRotationKD);
        autoAlignPID.enableContinuousInput(0, 2 * Math.PI);
        passPID = new PIDController(SwerveConstants.autoAlignRotationKP + 1, SwerveConstants.autoAlignRotationKI,
                SwerveConstants.autoAlignRotationKD);
        passPID.enableContinuousInput(0, 2 * Math.PI); 

        // Using last year's default deviations, need to tune

        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics, getYaw(), getModulePositions(),
                new Pose2d(), SwerveConstants.autostateStdDevs, VecBuilder.fill(0, 0, 0));

        poseLookup = new RobotPoseLookup();

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
                        new ReplanningConfig(false, false)),
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

    public void setLastMovingYaw(double value){
        lastMovingYaw = value;
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
            //shooterCenterVision.setPriorityID(LimelightConstants.blueSpeakerTag, LimelightConstants.shooterCenter.name);
        } else {
            shooterLeftVision.setPriorityID(LimelightConstants.redSpeakerTag, LimelightConstants.shooterLeft.name);
            shooterRightVision.setPriorityID(LimelightConstants.redSpeakerTag, LimelightConstants.shooterRight.name);
            //shooterCenterVision.setPriorityID(LimelightConstants.redSpeakerTag, LimelightConstants.shooterCenter.name);

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
        field2d.setRobotPose(getPose());
        
        Logger.recordOutput("Swerve/ChassisSpeeds", new Translation2d(ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getYaw()).vxMetersPerSecond, ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getYaw()).vyMetersPerSecond));
        if (DriverStation.isDisabled()) {
            for (SwerveModule mod : modules) {
                mod.stop();
            }
        }

        //shooterCenterVision.SetRobotOrientation(getYaw());
        shooterLeftVision.SetRobotOrientation(getYaw());
        shooterRightVision.SetRobotOrientation(getYaw());
        //IntakeTagVision.SetRobotOrientation(getYaw());

        addVisionMG2(shooterLeftVision);
        addVisionMG2(shooterRightVision);
        //addVisionMG2(IntakeTagVision);
        //addVisionMG2(shooterCenterVision)//;

        //shooterCenterVision.tagThrowOut(LimelightConstants.filtertags);
        //shooterLeftVision.tagThrowOut(//LimelightConstants.filtertags);
        shooterRightVision.tagThrowOut(LimelightConstants.filtertags);
        //IntakeTagVision.tagThrowOut(LimelightConstants.filtertags);

        // 3015 code
        // poseLookup.addPose(getPose());
        // if((shooterLeftVision.getTV() && shooterLeftVision.getDistToTag() <=4 ) ||
        // (shooterRightVision.getTV() && shooterRightVision.getDistToTag() <=4)){
        // correctOdom();
        // }

        /*
         * // Update PoseEstimator if at least 1 tag is in view
         * if (shooterRightVision.getBotPose().getY() <= FieldConstants.fieldWidth &&
         * shooterRightVision.getBotPose().getY() >= 0 &&
         * shooterRightVision.getBotPose().getX() <= FieldConstants.fieldLength &&
         * shooterRightVision.getBotPose().getX() >= 0 &&
         * Math.abs(shooterRightVision.getBotPose().getY()) != 0) {
         * // standard deviations are (distance to nearest apriltag)/2 for x and y and
         * 10
         * // degrees for theta
         * poseEstimator.addVisionMeasurement((shooterRightVision.getBotPose()),
         * (shooterRightVision.getPoseTimestamp()),
         * VecBuilder.fill(getStdDev(shooterRightVision.getDistToTag()),
         * getStdDev(shooterRightVision.getDistToTag()), Units.degreesToRadians(60)));
         * Logger.recordOutput("shooterrightvisiondist",
         * shooterRightVision.getDistToTag());
         * }
         * 
         * if (shooterLeftVision.getBotPose().getY() <= FieldConstants.fieldWidth &&
         * shooterLeftVision.getBotPose().getY() >= 0 &&
         * shooterLeftVision.getBotPose().getX() <= FieldConstants.fieldLength &&
         * shooterLeftVision.getBotPose().getX() >= 0 &&
         * Math.abs(shooterLeftVision.getBotPose().getY()) != 0) {
         * poseEstimator.addVisionMeasurement((shooterLeftVision.getBotPose()),
         * (shooterLeftVision.getPoseTimestamp()),
         * VecBuilder.fill(getStdDev(shooterLeftVision.getDistToTag()),
         * getStdDev(shooterLeftVision.getDistToTag()),
         * Units.degreesToRadians(60)));
         * Logger.recordOutput("shooterleftvisiondist",
         * shooterLeftVision.getDistToTag());
         * 
         * }
         */

    }

    // public double getStdDev(double dist) {
    // return DriverStation.isAutonomous() ? dist/LimelightConstants.autostdDev :
    // dist/LimelightConstants.telestdDev;
    // }

    public double getStdDev(double dist) {
        return DriverStation.isAutonomous() ? dist / LimelightConstants.autostdDev : Math.pow(dist, 2) / 2;
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
     * @param translation    desired x and y speeds of the swerve drive in meters
     *                       per
     *                       second
     * @param rotation       desired rotation speed of the swerve drive in radians
     *                       per second
     * @param fieldRelative  whether the values should be field relative or not
     * @param angleToSpeaker in radians
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean snapToAmp,
            boolean autoAlign, double autoAlignAngle, boolean rotateToNote, double tx) {
                
        // Rotation2d ampVal =
        // BobcatUtil.isBlue()?Constants.FieldConstants.blueAmpCenter.getRotation() :
        // Constants.FieldConstants.redAmpCenter.getRotation();
        double ampVal = BobcatUtil.isBlue() ? -Math.PI / 2 : Math.PI / 2;
        Logger.recordOutput("AmpAlign/ampVal", ampVal);

        ChassisSpeeds desiredSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getYaw())
                : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation);

        if (autoAlign) {
            desiredSpeeds.omegaRadiansPerSecond = autoAlignPID.calculate(get0to2Pi(getYaw().getRadians()),
                    get0to2Pi(autoAlignAngle));
            Logger.recordOutput("Swerve/AutoAlignPID/setpoint", get0to2Pi(autoAlignAngle));
            Logger.recordOutput("Swerve/AutoAlignPID/measurement", get0to2Pi(getYaw().getRadians()));
            Logger.recordOutput("Swerve/AutoAlignPID/error", autoAlignPID.getPositionError());
            lastMovingYaw = getYaw().getRadians();
        } else if (snapToAmp) {
            desiredSpeeds.omegaRadiansPerSecond = autoAlignPID.calculate(get0to2Pi(getYaw().getRadians()),
                    ampVal);
            lastMovingYaw = getYaw().getRadians();
        }else if(rotateToNote){
            
            desiredSpeeds.omegaRadiansPerSecond = autoAlignPID.calculate(tx/1.75,0);
            lastMovingYaw = getYaw().getRadians();
        } else {
            if (rotation == 0) {
                timer.reset();
                timer.start();
                if (rotating) {
                    
                    lastMovingYaw = getYaw().getRadians();
                    if (timer.hasElapsed(0.25)) {
                        rotating = false;
                        timer.stop();    
                    }
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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxModuleSpeed);

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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxModuleSpeed);

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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxModuleSpeed);

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

    //boolean <- pronnounced 'bolly-un'
    
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

    // /**
    //  * 
    //  * @return DISTANCE TO SPEAKER OF CURRENT ALLIANCE IN METERS :D
    //  */
    // @AutoLogOutput
    // public double getDistanceToSpeaker() {
    //     if (BobcatUtil.getAlliance() == Alliance.Blue) {
    //         if (shooterLeftVision.getID() == LimelightConstants.blueSpeakerTag
    //                 && shooterRightVision.getID() == LimelightConstants.blueSpeakerTag) {
    //             return (shooterLeftVision.getDistToTag() + shooterRightVision.getDistToTag()) / 2;
    //         } else {
    //             return getPose().getTranslation().getDistance(FieldConstants.blueSpeakerPose);
    //         }
    //     } else {
    //         if (shooterLeftVision.getID() == LimelightConstants.redSpeakerTag
    //                 && shooterRightVision.getID() == LimelightConstants.redSpeakerTag) {
    //             return (shooterLeftVision.getDistToTag() + shooterRightVision.getDistToTag()) / 2;
    //         } else {
    //             return getPose().getTranslation().getDistance(FieldConstants.redSpeakerPose);
    //         }
    //     }
    // }

    public double getDistanceToSpeakerForSpivit() {
        if (BobcatUtil.getAlliance() == Alliance.Blue) {
            Logger.recordOutput("DistanceToSpeaker/Pose", getPose().getTranslation().getDistance(FieldConstants.blueSpeakerPoseSpivit));
            //Logger.recordOutput("DistanceToSpeaker/SingleTag", shooterCenterVision.getDistToTag() + 0.181);

            //if (shooterCenterVision.getID() == LimelightConstants.blueSpeakerTag
            //        && (shooterCenterVision.getDistToTag() +
            //                0.181) <= LimelightConstants.throwoutDist) {
            //    return (shooterCenterVision.getDistToTag())
            //            + 0.181;
            //} else {
                return getPose().getTranslation().getDistance(FieldConstants.blueSpeakerPoseSpivit);
           // }
        } else {
            Logger.recordOutput("SpeakerAlign/SpivitDist/Pose/odometry", getPose().getTranslation().getDistance(FieldConstants.redSpeakerPoseSpivit));
           // Logger.recordOutput("SpeakerAlign/SpivitDist/Pose/SingleTag", shooterCenterVision.getDistToTag() + 0.181);
           // if (shooterCenterVision.getID() == LimelightConstants.redSpeakerTag
           //         && (shooterCenterVision.getDistToTag() +
           //                 0.181) <= LimelightConstants.throwoutDist) {
           //     return (shooterCenterVision.getDistToTag())
           //             + 0.181;
           // } else {
                return getPose().getTranslation().getDistance(FieldConstants.redSpeakerPoseSpivit);
           // }

        }
    }

    public Translation2d getTranslationToSpeaker() {
        return BobcatUtil.getAlliance() == Alliance.Blue
                ? FieldConstants.blueSpeakerPose.minus(getPose().getTranslation())
                : FieldConstants.redSpeakerPose.minus(getPose().getTranslation());
    }


    /**
     * 
     * @return the area where we want to pass, should be around the amp zone
     */
    public Translation2d getTranslationToPassArea() {
        return BobcatUtil.getAlliance() == Alliance.Blue
                ? FieldConstants.bluePassPose.minus(getPose().getTranslation())
                : FieldConstants.redPassPose.minus(getPose().getTranslation());
    }

    public double getRotationToNote(){
        if (IntakeVision.getTV()){
            return IntakeVision.getTX().getRadians();
        }else{
            return 0;
        }
    }


    // /**
    //  * 
    //  * 
    //  * r^2 = 0.983
    //  */
    // public double calcAngleBasedOnRealRegression() {
    //     double distance = getDistanceToSpeaker();
    //     Logger.recordOutput("Spivit/DesiredAngle", 291 * Math.pow(distance, -0.074));
    //     return 291 * Math.pow(distance, -0.074);

    // }

    // /**
    //  * 
    //  * 
    //  * r^2 = 0.996
    //  */
    // public double calcAngleBasedOnEstimatorRegression() {
    //     double distance = getDistanceToSpeaker();
    //     return 291 * Math.pow(distance, -0.0762);
    // }

    public double calcAngleBasedOnHashMap() {
        double distance = getDistanceToSpeakerForSpivit();
        // double angle = Math.abs(getAngleToSpeakerTagAuto().getDegrees());
        Logger.recordOutput("Spivit/DesiredAngle", ShooterConstants.spivitAngles.get(distance));
        return ShooterConstants.spivitAngles.get(distance);
        // Logger.recordOutput("Spivit/DesiredAngle",
        // ShooterConstants.spivitBiLinearInterpolation.interpolate(distance, angle));
        // return ShooterConstants.spivitBiLinearInterpolation.interpolate(distance,
        // angle);
    }

    public double calcAngleBasedOnHashMap(double dist) {
        double distance = dist;
        // double angle = Math.abs(getAngleToSpeakerTagAuto().getDegrees());
        Logger.recordOutput("Spivit/DesiredAngle", ShooterConstants.spivitAngles.get(distance));
        return ShooterConstants.spivitAngles.get(distance);
        // Logger.recordOutput("Spivit/DesiredAngle",
        // ShooterConstants.spivitBiLinearInterpolation.interpolate(distance, angle));
        // return ShooterConstants.spivitBiLinearInterpolation.interpolate(distance,
        // angle);
    }

    public double get0to2Pi(double rad) {
        rad = rad % (2 * Math.PI);
        if (rad < (2 * Math.PI)) {
            rad += (2 * Math.PI);
        } // should this be here?
        return rad;
    }

    public boolean aligned() {
        if (DriverStation.isTeleop()) {
            return Math
                    .abs(Math.toDegrees(rotationPID.getPositionError())) <= SwerveConstants.rotationToleranceAlignment;
        } else {
            if (BobcatUtil.getAlliance() == Alliance.Blue) {
                return Math.abs(ppRotationOverride.getRadians() - getYaw().getRadians()) <= 1;
            } else {
                return Math.abs(ppRotationOverride.getRadians()
                        - get0to2Pi(getYaw().rotateBy(Rotation2d.fromDegrees(180)).getRadians())) <= 1;
            }
        }
    }

    public boolean alignedLEDS(double spivitAngle) {
        if(spivitAngle < ShooterConstants.ampPosition){
            return Math.abs(Math.toDegrees(autoAlignPID.getPositionError())) <= 3; //3 degrees of tolerance if we're far away
        }else{
            return Math.abs(Math.toDegrees(autoAlignPID.getPositionError())) <= 5; //5 degrees of tolerance if we're close
        }
    }

    public boolean aligned(Rotation2d angle) {
        if (BobcatUtil.getAlliance() == Alliance.Blue) {
            return Math.abs(angle.getRadians() - getYaw().getRadians()) <= Math.toRadians(2.5); // TODO Formerly 1 radian -_-
        } else {
            return Math.abs(angle.getRadians() - getYaw().getRadians()) <= Math.toRadians(2.5);
        }
    }

    /**
     * 
     * @return the angle to the speaker in radians
     * 
     */
    public double getAngleToSpeaker() {
        Translation2d speaker = getTranslationToSpeaker();
        return Math.atan(speaker.getY() / speaker.getX());
    }

    /**
     * 
     * @return the angle to the pass area in radians
     * 
     */
    public double getAngleToPassArea() {
        Translation2d passArea = getTranslationToPassArea();
        return Math.atan(passArea.getY() / passArea.getX());
    }

    // /**
    // * @return angle to the speaker biased for outta the way
    // */
    // public double getAngleToSpeakerBiased() {
    // Translation2d speaker = getTranslationToSpeaker().plus(new Translation2d(0,
    // 0.3));
    // return Math.atan(speaker.getY() / speaker.getX());
    // }

    private Rotation2d lastValue = new Rotation2d();

    public Rotation2d getAngleToSpeakerApriltag() {

        Rotation2d odometryValue = Rotation2d.fromRadians(getAngleToSpeaker());

       // if (shooterCenterVision.getTV()
       //         && getDistanceToSpeakerForSpivit() < ShooterConstants.holonomicAprilTagThrowoutDistance) {
//
       //     lastValue = Rotation2d.fromRadians(get0to2Pi((getYaw().getRadians()
       //             - (((shooterCenterVision.getTX().getRadians()))))));
       //     Logger.recordOutput("Autoalign/Using Tag", true);
       //     return lastValue;
       // }
        //else if
        if (Math.abs(odometryValue.minus(lastValue).getDegrees()) < 2.5) { // if we dont see both tags and odometry
                                                                                // is within
            // 2.5 degrees of the last reported tag value
            Logger.recordOutput("Autoalign/Using Tag", false);
            return lastValue;

        } else {
            Logger.recordOutput("Autoalign/Using Tag", false);
            return odometryValue; // the previous statement does nothing because of this
        }
    }

    public Rotation2d getAngleToSpeakerTagAuto() {
        Rotation2d odometryValue = Rotation2d.fromRadians(getAngleToSpeaker());
        Logger.recordOutput("SpeakerAlign/Angle/odometry", odometryValue);
       // if(shooterCenterVision.getTV()){
       // Logger.recordOutput("SpeakerAlign/Angle/LLCenter", shooterCenterVision.getTX());
       // }

        // Rotation2d leftLimeValue = shooterLeftVision.getTV() ?
        // Rotation2d.fromRadians(get0to2Pi(getYaw().getRadians() -
        // shooterLeftVision.getTX().getRadians())) : null;

        // Rotation2d rightLimeValue = shooterRightVision.getTV() ?
        // Rotation2d.fromRadians(get0to2Pi(getYaw().getRadians() -
        // shooterRightVision.getTX().getRadians())) : null;

        // Rotation2d total = odometryValue.plus(leftLimeValue != null ? leftLimeValue :
        // new Rotation2d()).plus(rightLimeValue != null ? rightLimeValue : new
        // Rotation2d());

        // int divisor = 1;
        // if (leftLimeValue != null) {
        // divisor++;
        // }
        // if (rightLimeValue != null) {
        // divisor++;
        // }
        // return total.div(divisor);
        return odometryValue;
    }

    /**
     * 
     * @return distance to amp along x axis in meters
     * TODO might need to invert
     */
    public double getXDistanceToAmp(){
        return BobcatUtil.isBlue() ? getPose().getX() - FieldConstants.blueAmpXPos : FieldConstants.redAmpXPos - getPose().getX();
    }


    public void setLimeLEDS(boolean on) {
        shooterLeftVision.setLEDS(on);
        shooterRightVision.setLEDS(on);
        IntakeVision.setLEDS(on);
    }

    public void addVision(Vision vision) {
        Matrix<N3, N1> stdDev;
        Matrix<N3, N1> truststdDev = DriverStation.isAutonomous() ? LimelightConstants.trustautostdDev
                : LimelightConstants.trusttelestdDev;
        Matrix<N3, N1> regstdDev = DriverStation.isAutonomous() ? LimelightConstants.regautostdDev
                : LimelightConstants.regtelestdDev;
        Logger.recordOutput("Pose/" + vision.getLimelightName(), vision.getBotPose());

        // stdDev = regstdDev;
        if (vision.getPoseEstimate().tagCount >= 2) {
            stdDev = truststdDev;
        } else {
            stdDev = regstdDev;
        }

        if (vision.getPoseValid(getYaw())) {
            poseEstimator.addVisionMeasurement(vision.getBotPose(), vision.getPoseTimestampMG2(), stdDev);
            // System.out.println("yes " + vision.getLimelightName() + " " +
            // Timer.getFPGATimestamp());
        }

    }

    public void addVisionMG2(Vision vision) {

        Matrix<N3, N1> stdDev;
        Matrix<N3, N1> truststdDev = DriverStation.isAutonomous() ? LimelightConstants.trustautostdDev
                : LimelightConstants.trusttelestdDev;
        Matrix<N3, N1> regstdDev = DriverStation.isAutonomous() ? LimelightConstants.regautostdDev
                : LimelightConstants.regtelestdDev;
        Logger.recordOutput("Pose/" + vision.getLimelightName(), vision.getBotPoseMG2());

        // stdDev = regstdDev;
        if (vision.getPoseEstimateMG2().tagCount >= 2) {
            stdDev = truststdDev;
        } else {
            stdDev = regstdDev;
        }

        if (vision.getPoseValidMG2(getYaw())) {
            poseEstimator.addVisionMeasurement(vision.getBotPoseMG2(), vision.getPoseTimestampMG2(), stdDev);
            // System.out.println("yes " + vision.getLimelightName() + " " +
            // Timer.getFPGATimestamp());
        }

    }

    public void correctOdom() {
        Pose2d leftBotPose = null;
        double leftPoseTimestamp = shooterLeftVision.getPoseTimestamp();
        double leftAprilTagDist = 0;
        Pose2d rightBotPose = null;
        double rightPoseTimestamp = shooterRightVision.getPoseTimestamp();
        double rightAprilTagDist = 0;

        Pose2d robotAtLeftCapture = poseLookup.lookup(leftPoseTimestamp);
        Pose2d robotAtRightCapture = poseLookup.lookup(rightPoseTimestamp);

        if (shooterLeftVision.getTV()) {
            Pose2d botpose = shooterLeftVision.getBotPose();

            if (botpose.getX() > 0.1
                    && botpose.getX() < FieldConstants.fieldLength - 0.1
                    && botpose.getY() > 0.1
                    && botpose.getY() < FieldConstants.fieldWidth - 1) {
                leftBotPose = botpose;
                leftAprilTagDist = shooterLeftVision.getDistToTag();
            }
        }

        if (shooterRightVision.getTV()) {
            Pose2d botpose = shooterRightVision.getBotPose();

            if (botpose.getX() > 0.1
                    && botpose.getX() < FieldConstants.fieldLength - 0.1
                    && botpose.getY() > 0.1
                    && botpose.getY() < FieldConstants.fieldWidth - 1) {
                rightBotPose = botpose;
                rightAprilTagDist = shooterRightVision.getDistToTag();
            }
        }

        Pose2d correctionPose = null;
        Pose2d robotAtCorrectionPose = null;
        double correctionTimestamp = 0;
        Matrix<N3, N1> correctionDevs = null;
        Matrix<N3, N1> truststdDev = DriverStation.isAutonomous() ? LimelightConstants.trustautostdDev
                : LimelightConstants.trusttelestdDev;
        Matrix<N3, N1> regstdDev = DriverStation.isAutonomous() ? LimelightConstants.regautostdDev
                : LimelightConstants.regtelestdDev;

        if (leftBotPose != null && rightBotPose != null) {
            // Left and right have poses
            Pose2d leftToRightDiff = leftBotPose.relativeTo(rightBotPose);
            if (leftToRightDiff.getTranslation().getNorm() < 0.3
                    && (Math.abs(leftToRightDiff.getRotation().getDegrees()) < 15)) {
                // They agree
                correctionPose = leftBotPose.interpolate(rightBotPose, 0.5);
                robotAtCorrectionPose = robotAtLeftCapture.interpolate(robotAtRightCapture, 0.5);
                correctionTimestamp = (leftPoseTimestamp + rightPoseTimestamp) / 2.0;
                // correctionDevs = VecBuilder.fill(((leftAprilTagDist +
                // rightAprilTagDist)/2)/stdDev, ((leftAprilTagDist +
                // rightAprilTagDist)/2)/stdDev, 99999);
                correctionDevs = truststdDev;
            } else {
                // They don't agree
                Pose2d leftDiff = leftBotPose.relativeTo(robotAtLeftCapture);
                Pose2d rightDiff = rightBotPose.relativeTo(robotAtRightCapture);
                double leftDist = leftDiff.getTranslation().getNorm();
                double rightDist = rightDiff.getTranslation().getNorm();

                if ((leftDist < 2.0) && leftDist <= rightDist) {
                    // Left closest
                    if (Math.abs(leftDiff.getRotation().getDegrees()) < 15) {
                        correctionPose = leftBotPose;
                        robotAtCorrectionPose = robotAtLeftCapture;
                        correctionTimestamp = leftPoseTimestamp;
                        // correctionDevs = VecBuilder.fill(leftAprilTagDist/stdDev,
                        // leftAprilTagDist/stdDev, 99999);
                        correctionDevs = regstdDev;
                    }
                } else if ((rightDist < 2.0) && rightDist <= leftDist) {
                    // Right closest
                    if (Math.abs(rightDiff.getRotation().getDegrees()) < 15) {
                        correctionPose = rightBotPose;
                        robotAtCorrectionPose = robotAtRightCapture;
                        correctionTimestamp = rightPoseTimestamp;
                        // correctionDevs = VecBuilder.fill(rightAprilTagDist/stdDev,
                        // rightAprilTagDist/stdDev, 99999);
                        correctionDevs = regstdDev;

                    }
                }
            }
        } else if (leftBotPose != null) {
            Pose2d leftDiff = leftBotPose.relativeTo(robotAtLeftCapture);
            double leftDist = leftDiff.getTranslation().getNorm();

            if (leftDist < 2.0) {
                if (Math.abs(leftDiff.getRotation().getDegrees()) < 15) {
                    correctionPose = leftBotPose;
                    robotAtCorrectionPose = robotAtLeftCapture;
                    correctionTimestamp = leftPoseTimestamp;
                    // correctionDevs = VecBuilder.fill(leftAprilTagDist/stdDev,
                    // leftAprilTagDist/stdDev, 99999);
                    correctionDevs = regstdDev;

                }
            }
        } else if (rightBotPose != null) {
            Pose2d rightDiff = rightBotPose.relativeTo(robotAtRightCapture);
            double rightDist = rightDiff.getTranslation().getNorm();

            if (rightDist < 2.0) {
                if (Math.abs(rightDiff.getRotation().getDegrees()) < 15) {
                    correctionPose = rightBotPose;
                    robotAtCorrectionPose = robotAtRightCapture;
                    correctionTimestamp = rightPoseTimestamp;
                    // correctionDevs = VecBuilder.fill(rightAprilTagDist/stdDev,
                    // rightAprilTagDist/stdDev, 99999);
                    correctionDevs = regstdDev;
                }
            }
        }

        if (correctionPose != null) {
            poseEstimator.addVisionMeasurement(
                    new Pose2d(correctionPose.getTranslation(),
                            BobcatUtil.isBlue() ? new Rotation2d(get0to2Pi(getYaw().getRadians()))
                                    : new Rotation2d(get0to2Pi(getYaw().getRadians()))
                                            .rotateBy(Rotation2d.fromDegrees(180))),
                    correctionTimestamp,
                    correctionDevs);
        }
    }

    /**
     * 
     * @return [0] field relative holo angle
     * @return [1] spivit angle
     */
    public double[] getShootWhileMoveBallistics(double spivitAngle) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getYaw());
        Translation2d speakerPose = BobcatUtil.isRed() ? FieldConstants.redSpeakerPoseSpivit
                : FieldConstants.blueSpeakerPoseSpivit;
        Translation2d robot = getPose().getTranslation();

        double real_spivit = spivitAngle - ShooterConstants.encoderOffsetFromHorizontal;

        // A lot of the code from this point forward is from here:
        // https://www.forrestthewoods.com/blog/solving_ballistic_trajectories/
        double G = 9.81;
        double target_pos_x = speakerPose.getX();
        double target_pos_y = FieldConstants.speakerHeight;
        double target_pos_z = speakerPose.getY();
        double target_vel_x = BobcatUtil.isRed() ? chassisSpeeds.vxMetersPerSecond : -chassisSpeeds.vxMetersPerSecond;
        double target_vel_y = 0;
        double target_vel_z = BobcatUtil.isRed() ? chassisSpeeds.vyMetersPerSecond : -chassisSpeeds.vyMetersPerSecond;
        double proj_pos_x = robot.getX();
        double proj_pos_y = Units.inchesToMeters(21) * Math.asin(Units.degreesToRadians(real_spivit))
                + Units.inchesToMeters(9);
        double proj_pos_z = robot.getY();
        double proj_speed = ShooterConstants.noteIdealExitVelocityMPS;
        Translation2d linearFieldVelocity = new Translation2d(-target_vel_x, -target_vel_z);

        double A = proj_pos_x;
        double B = proj_pos_y;
        double C = proj_pos_z;
        double M = target_pos_x;
        double N = target_pos_y;
        double O = target_pos_z;
        double P = target_vel_x;
        double Q = target_vel_y;
        double R = target_vel_z;
        double S = proj_speed;

        double H = M - A;
        double J = O - C;
        double K = N - B;
        double L = -.5f * G;

        // Quartic Coeffecients
        double c0 = G * G;
        double c1 = 0;
        double c2 = (P * P + R * R) + (K * -G) - S * S;
        double c3 = 2 * (H * P + K * R);
        double c4 = K * K + H * H + J * J;

        double[] q_sols = new double[5];
        q_sols = Quartic.solveQuartic(c0, c1, c2, c3, c4);
        double[] times = new double[4];
        for (int i = 0; i < times.length; i++) {
            times[i] = q_sols[i];
        }

        Arrays.sort(times);

        Translation3d[] solution_poses = new Translation3d[2];
        solution_poses[0] = new Translation3d();
        solution_poses[1] = new Translation3d();
        int num_sols = 0;

        for (int i = 0; i < times.length; i++) {
            double t = times[i];

            if (t <= 0 || Double.isNaN(t)) {
                continue;
            }

            solution_poses[num_sols] = new Translation3d(
                    (float) ((H + P * t) / t),
                    (float) ((K + Q * t - L * t * t) / t),
                    (float) ((J + R * t) / t));
            num_sols++;
        }
        Translation3d sol_pose = solution_poses[0];
        Rotation2d holo_align_angle = new Rotation2d(sol_pose.getX(), sol_pose.getZ());

        double[] ret_val = new double[2];

        ret_val[0] = get0to2Pi(holo_align_angle.getRadians());
        // ret_val[1] = new Rotation2d(Math.hypot(sol_pose.getX(), sol_pose.getZ()),
        // sol_pose.getY()).getDegrees() + ShooterConstants.encoderOffsetFromHorizontal;

        double[] shotTime = new double[2];
        int numOfTimes = 0;

        for (int i = 0; i < times.length; i++) {
            double t = times[i];

            if (t <= 0 || Double.isNaN(t)) {
                continue;
            }

            shotTime[numOfTimes] = t;
            numOfTimes++;
        }

        Rotation2d speakerToRobotAngle = robot.minus(speakerPose).getAngle();
        Translation2d tangentialVelocity = linearFieldVelocity.rotateBy(speakerToRobotAngle.unaryMinus());
        // Positive when velocity is away from speaker
        double radialComponent = tangentialVelocity.getX();
        // Positive when traveling CCW about speaker
        double tangentialComponent = tangentialVelocity.getY();

        // Add robot velocity to raw shot speed
        double rawDistToGoal = getDistanceToSpeakerForSpivit();
        double shotSpeed = rawDistToGoal / shotTime[0] + radialComponent;
        if (shotSpeed <= 0.0)
            shotSpeed = 0.0;
        double effectiveDist = shotTime[0] * Math.hypot(tangentialComponent, shotSpeed);

        ret_val[1] = calcAngleBasedOnHashMap(effectiveDist);
        if (Math.abs(tangentialComponent) > 2) {
            ret_val[1] += (1.2*tangentialComponent);
        }

        return ret_val;
    }

    public double velocityTowardsPassingSpot() {
        Translation2d speakerPose;
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getYaw());
        Translation2d linearFieldVelocity = new Translation2d(-(BobcatUtil.isRed() ? chassisSpeeds.vxMetersPerSecond : -chassisSpeeds.vxMetersPerSecond), -(BobcatUtil.isRed() ? chassisSpeeds.vyMetersPerSecond : -chassisSpeeds.vyMetersPerSecond));
        if (BobcatUtil.isBlue()) {
            speakerPose = FieldConstants.bluePassPose;
        } else {
            speakerPose = FieldConstants.redPassPose;
        }
        Rotation2d speakerToRobotAngle = getPose().getTranslation().minus(speakerPose).getAngle();
        Translation2d tangentialVelocity = linearFieldVelocity.rotateBy(speakerToRobotAngle.unaryMinus());
        // Positive when velocity is away from speaker
        double radialComponent = tangentialVelocity.getX();
        // Positive when traveling CCW about speaker
        double tangentialComponent = tangentialVelocity.getY();

        return -tangentialComponent*1.25    ;
    }

    public void SetSmoothieAutoIsAligned(boolean aligned){
        isSmoothieAligned = aligned;
    }

    public boolean isSmoothieAutoAligned(){
        return isSmoothieAligned;
    }

}