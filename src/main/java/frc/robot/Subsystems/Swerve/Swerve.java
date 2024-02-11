package frc.robot.Subsystems.Swerve;

import java.util.Optional;
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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.GeometryUtils;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Vision.Vision;

public class Swerve extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveModule[] modules;
    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator PoseEstimator;
    private final Vision shooterLeftVision;
    private final Vision shooterRightVision;
    private final Vision IntakeVision;

    
    private final double[] swerveModuleStates = new double[8];
    private final double[] desiredSwerveModuleStates = new double[8];

    private final PIDController rotationPID;
    private double lastMovingYaw = 0.0;
    private boolean rotating = false;

    private final LoggedDashboardNumber driveToPoseX = new LoggedDashboardNumber("desired x");
    private final LoggedDashboardNumber driveToPoseY = new LoggedDashboardNumber("desired y");    
    Pose2d desiredPose = new Pose2d();
    private Rotation2d lastYaw = new Rotation2d();
    

    public Swerve(GyroIO gyroIO, SwerveModuleIO flIO, SwerveModuleIO frIO, SwerveModuleIO blIO, SwerveModuleIO brIO, Vision intakeVision, Vision shooterLeftVision, Vision shooterRightVision) {
        
        //Notifier for odometry updates, didn't work very well in my testing

        // class updatePose implements Runnable{
        //     @Override
        //     public void run(){
        //     //Update PoseEstimator based on odometry
        //     // PoseEstimator.update(getYaw(), getModulePositions());
        //     PoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());

        //     //Update PoseEstimator if at least 1 tag is in view
        //     if (LimelightHelpers.getTV(limelightfront)){
        //     //standard deviations are (distance to nearest apriltag)/2 for x and y and 10 degrees for theta
        //     PoseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue(limelightfront), (Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline(limelightfront)/1000.0) - (LimelightHelpers.getLatency_Capture(limelightfront)/1000.0)),VecBuilder.fill(getDistance(limelightfront)/2, getDistance(limelightfront)/2, Units.degreesToRadians(10)));
        //     }
        //     if (LimelightHelpers.getTV(limelightback)){
        //     PoseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue(limelightback), (Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline(limelightback)/1000.0) - (LimelightHelpers.getLatency_Capture(limelightback)/1000.0)), VecBuilder.fill(getDistance(limelightback)/2, getDistance(limelightback)/2, Units.degreesToRadians(10)));
        //     }
            
        
        // }
            
        // }

        // Runnable runnable = new updatePose();
        // Notifier notifier = new Notifier(runnable);
        // double runnablePeriod = 0.02;
        // notifier.startPeriodic(runnablePeriod);
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

        odometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getYaw(), getModulePositions());

        rotationPID = new PIDController(SwerveConstants.teleopRotationKP, SwerveConstants.teleopRotationKI, SwerveConstants.teleopRotationKD);
        
        //Using last year's default deviations, need to tune
        PoseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics, getYaw().times(-1), getModulePositions(), new Pose2d(), SwerveConstants.stateStdDevs, VecBuilder.fill(0,0,0));


        AutoBuilder.configureHolonomic(
                this::getPoseEstimation,
                this::resetPoseEstimator,
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
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
                PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    }
    public Optional<Rotation2d> getRotationTargetOverride(){
        if(getRotationTarget() != null){
            return Optional.of(getRotationTarget());
        }else{
            return Optional.empty();
        }
    }

    public Rotation2d getRotationTarget(){
        return null; //TODO: the rotation2d this returns will override the one in pathplanner, if null, the default pathplanner rotation will be used
    }
    

    public void periodic(){
        gyroIO.updateInputs(gyroInputs);
        Logger.recordOutput("Swerve/YawSetpoint", lastMovingYaw);
        Logger.recordOutput("Swerve/CurrentYaw", getYaw().getRadians());

        Logger.processInputs("Swerve/Gyro", gyroInputs);
        for (SwerveModule module : modules) {
            module.periodic();
        }

        if (gyroInputs.connected) { // Use gyro when connected
            Rotation2d yaw = getYaw();
            odometry.update(yaw, getModulePositions());
            lastYaw = yaw;
        } else { // If disconnected or sim, use angular velocity
            Rotation2d yaw = lastYaw.plus(Rotation2d.fromRadians(getChassisSpeeds().omegaRadiansPerSecond * Constants.loopPeriodSecs));
            odometry.update(yaw, getModulePositions());
            lastYaw = yaw;
        }

        for (SwerveModule mod : modules) {
            SmartDashboard.putNumber("Mod " + mod.index + " Angle", mod.getRawCanCoder());
            desiredSwerveModuleStates[mod.index * 2 + 1] = mod.getDesiredState().speedMetersPerSecond;
            desiredSwerveModuleStates[mod.index * 2] = mod.getDesiredState().angle.getDegrees();
            swerveModuleStates[mod.index * 2 + 1] = mod.getState().speedMetersPerSecond;
            swerveModuleStates[mod.index * 2] = mod.getState().angle.getDegrees();
        }
        
        Logger.recordOutput("Swerve/Rotation", odometry.getPoseMeters().getRotation().getDegrees());
        Logger.recordOutput("Swerve/DesiredModuleStates", desiredSwerveModuleStates);
        Logger.recordOutput("Swerve/ModuleStates", swerveModuleStates);
        Logger.recordOutput("Swerve/OdometryPose", getPose());
        Logger.recordOutput("Swerve/PoseEstimation", getPoseEstimation());
        Logger.recordOutput("Swerve/Pose3d", getPose3d());
        if (DriverStation.isDisabled()) {
            for (SwerveModule mod : modules) {
                mod.stop();
            }
        }
        desiredPose = new Pose2d(driveToPoseX.get(), driveToPoseY.get(), new Rotation2d());
        Logger.recordOutput("Swerve/DesiredPose", desiredPose);


        
        //Update PoseEstimator based on odometry
        // PoseEstimator.update(getYaw(), getModulePositions());
            PoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw().times(-1), getModulePositions());

            //Update PoseEstimator if at least 1 tag is in view
            if (shooterRightVision.getTV()){
                //standard deviations are (distance to nearest apriltag)/2 for x and y and 10 degrees for theta
                PoseEstimator.addVisionMeasurement((shooterRightVision.getBotPose()), (shooterRightVision.getPoseTimestamp()),VecBuilder.fill(shooterRightVision.getDistToTag()/2, shooterRightVision.getDistToTag()/2, Units.degreesToRadians(25)));
            
            }
            if (shooterLeftVision.getTV()){
                PoseEstimator.addVisionMeasurement((shooterLeftVision.getBotPose()), (shooterLeftVision.getPoseTimestamp()),VecBuilder.fill(shooterLeftVision.getDistToTag()/2, shooterLeftVision.getDistToTag()/2, Units.degreesToRadians(25)));
            }



    }


    public Pose2d getPathfindingPose(){
        
        return desiredPose;
    }

    /**
     * Gets the current yaw of the gyro or the estimated yaw if the gyro is disconnected
     * @return current yaw of the gyro
     */
    public Rotation2d getYaw() {
        if (gyroInputs.connected) { // Use gyro when connected
            return Rotation2d.fromDegrees(gyroInputs.yawPositionDeg);
        } else { // If disconnected or sim, use angular velocity
            return lastYaw;
        }
    }



    /**
     * Gets the current pitch of the gyro
     * @return current pitch of the gyro
     */
    public double getPitch() {
        return gyroInputs.pitchPositionDeg;
    }
    
    
    /**
     * Gets the current roll of the gyro
     * @return current roll of the gyro
     */
    public double getRoll() {
        return gyroInputs.rollPositionDeg;
    }

    /**
     * Makes the swerve drive move
     * @param translation desired x and y speeds of the swerve drive in meters per second
     * @param rotation desired rotation speed of the swerve drive in radians per second
     * @param fieldRelative whether the values should be field relative or not
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean snapToRotation) {
        
        ChassisSpeeds desiredSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getYaw())
                : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation);

        // desiredSpeeds = correctForDynamics(desiredSpeeds);

        if (snapToRotation) {
            desiredSpeeds.omegaRadiansPerSecond = rotationPID.calculate(getYaw().getRadians(), Math.PI/2);
        } else {
            if (rotation == 0) {
                if (rotating) {
                    rotating = false;
                    lastMovingYaw = getYaw().getRadians();
                }
                desiredSpeeds.omegaRadiansPerSecond = -rotationPID.calculate(getYaw().getRadians(), lastMovingYaw);
            } else {
                rotating = true;
            }
        }

        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : modules) {
            mod.setDesiredState(swerveModuleStates[mod.index]);
        }
    }

    /**
     * 
     * Make the swerve drive move
     * @param targetSpeeds the desired chassis speeds
     */
    public void drive(ChassisSpeeds targetSpeeds) {
        targetSpeeds = ChassisSpeeds.discretize(targetSpeeds, Constants.loopPeriodSecs);
        
        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : modules) {
            mod.setDesiredState(swerveModuleStates[mod.index]);
        }
    }

    /**
     * Sets all of the modules to their desired states
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
     * @return distance to nearest apriltag in meters
     */

    //  public double getDistance(String limelight) {
    //     // return PoseEstimator.getEstimatedPosition().getTranslation().getDistance(new Pose2d(LimelightHelpers.getTargetPose3d_RobotSpace(limelight).getX(), LimelightHelpers.getTargetPose3d_RobotSpace(limelight).getY()).getTranslation());
    //     //getting x distance to target
    //     return LimelightHelpers.getTargetPose_RobotSpace(limelight)[0];
    // }



    /**
     * Gets all of the current module states
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
     * @return current chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Gets the current pose, according to our odometry
     * @return current pose in meters
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Pose2d getPoseEstimation() {
        return PoseEstimator.getEstimatedPosition();
    }

    public Pose3d getPose3d(){
        return (new Pose3d(new Translation3d(PoseEstimator.getEstimatedPosition().getX(),PoseEstimator.getEstimatedPosition().getY(),0), new Rotation3d(-Units.degreesToRadians(getRoll()),-Units.degreesToRadians(getPitch()),getYaw().getRadians())));
    }



    /**
     * Resets our odometry to desired pose
     * @param pose pose to set odometry to
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void resetPoseEstimator(Pose2d pose) {
        PoseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
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

    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.04;
        // TODO test arbitrarily making this larger to see if it helps
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
      }

    /**
     * Stops the swerve drive
     */
    public void stop() {
        drive(new ChassisSpeeds());
    }

    public Command driveToPose(Pose2d pose){
        
        return AutoBuilder.pathfindToPose(pose, new PathConstraints(Constants.SwerveConstants.maxSpeed, Constants.SwerveConstants.maxAccel, Constants.SwerveConstants.maxAngularVelocity, Constants.SwerveConstants.maxAngularAcceleration));
    }

}