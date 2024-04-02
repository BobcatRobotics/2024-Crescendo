// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Util.BobcatUtil;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public boolean apriltagPipeline;

  public Vision(VisionIO io) {
    this.io = io;

    io.setLEDS(LEDMode.FORCEOFF);
    // io.setPipeline(Constants.LimelightConstants.apriltagPipelineIndex);

  }

  public void setLEDS(boolean on) {
    io.setLEDS(on ? LEDMode.FORCEBLINK : LEDMode.PIPELINECONTROL);
  }

  public void setCamMode(CamMode mode) {
    io.setCamMode(mode);
  }

  public double getTClass() {
    return inputs.tClass;
  }

  public boolean getTV() {
    return inputs.tv;
  }

  public double getID(){
    return inputs.fiducialID;
  }

  public void setPipeline(int id) {
    io.setPipeline(inputs.name, id);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Limelight" + inputs.name, inputs);

    apriltagPipeline = inputs.pipelineID == 0;

    Logger.recordOutput("note pose/note pose", getNotePose());
    // Logger.recordOutput("translation to note", getTranslationToTag((int) inputs.fiducialID));

  }

  public double getNoteY() {
    Logger.recordOutput("Limelight" + inputs.name + "/noteY",
        inputs.distanceToNote * Math.cos(Math.toRadians(90 - inputs.tx)));
    return inputs.distanceToNote * Math.cos(Math.toRadians(90 - inputs.tx));
  }

  public Pose2d getNotePose() {
    return new Pose2d(inputs.distanceToNote, getNoteY(), Rotation2d.fromDegrees(inputs.tx));
  }

  public Pose2d getBotPose() {
    Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue(inputs.name);
    Logger.recordOutput("Limelight" + inputs.name + "/pose", pose);
    return pose;
  }

  public Pose3d getBotPose3d() {
    Pose3d pose = LimelightHelpers.getBotPose3d_wpiBlue(inputs.name);
    Logger.recordOutput("Limelight" + inputs.name + "/Pose3d", pose);
    return pose;

  }

  public double getDistToTag() {
    //indexes don't match documentation with targetpose_robotspace
    // Logger.recordOutput("distanceToTagHypot", Math.hypot(LimelightHelpers.getTargetPose_RobotSpace(inputs.name)[0], LimelightHelpers.getTargetPose_RobotSpace(inputs.name)[2]));
    // return Math.hypot(Math.abs(LimelightHelpers.getTargetPose_RobotSpace(inputs.name)[0]), LimelightHelpers.getTargetPose_RobotSpace(inputs.name)[2]); // 0 is x, 2 is z 
    Logger.recordOutput("Limelight" + inputs.name + "/distanceToTagHypot", Math.hypot(LimelightHelpers.getCameraPose_TargetSpace(inputs.name)[0], LimelightHelpers.getCameraPose_TargetSpace(inputs.name)[2]));
    return Math.hypot(LimelightHelpers.getCameraPose_TargetSpace(inputs.name)[0], LimelightHelpers.getCameraPose_TargetSpace(inputs.name)[2]); // 0 is x, 2 is z 
    
  }

  public double getPoseTimestamp() {
    // return Timer.getFPGATimestamp() - ((LimelightHelpers.getLatency_Pipeline(inputs.name)+LimelightHelpers.getLatency_Capture(inputs.name)) / 1000.0);
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(inputs.name).timestampSeconds;
  }

  public LimelightHelpers.PoseEstimate getPoseEstimate() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(inputs.name);
  }


   public String getLimelightName(){
    return inputs.name;
   }


  public boolean getPoseValid(Rotation2d gyro) {
    Pose3d botpose = LimelightHelpers.getBotPose3d_wpiBlue(inputs.name);
    Logger.recordOutput("Pose3d/"+inputs.name, botpose);
    double diff = 0;

    double gyroval=gyro.getDegrees();
    gyroval = gyroval % (360);

    // gyroval=Units.radiansToDegrees(BobcatUtil.get0to2Pi(gyroval));
    double llrotation=botpose.toPose2d().getRotation().getDegrees();
    diff = BobcatUtil.isRed()? Math.abs(Math.abs(gyroval-llrotation)-180) : Math.abs(Math.abs(gyroval-llrotation));
    // double diff = 0;
    double z = botpose.getZ();
    double x = botpose.getX();
    double y = botpose.getY();
    Logger.recordOutput("LLDebug/"+inputs.name+" z val", z);
    Logger.recordOutput("LLDebug/"+inputs.name+" x val", x);
    Logger.recordOutput("LLDebug/"+inputs.name+" y val", y);
    Logger.recordOutput("LLDebug/"+inputs.name+" rdiff", diff);

    double ambiguity = 0;

    LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(inputs.name);

  //  if (botpose.getX()!=0 && botpose.getY()!=0){ 
  //    ambiguity = poseEstimate.rawFiducials[0].ambiguity; //not tested, may not work correctly as of 3/27
  //  }


    
    double tagDist = poseEstimate.avgTagDist;
    Logger.recordOutput("LLDebug/"+inputs.name+" avgTagDist", tagDist);
    Logger.recordOutput("LLDebug/"+inputs.name+" tagCount", poseEstimate.tagCount);


    // double tagDist=0;

    if( 
        (diff<LimelightConstants.rotationTolerance) && 
        (z<LimelightConstants.zDistThreshold) && 
        (ambiguity<LimelightConstants.poseAmbiguityThreshold) && 
        (tagDist<LimelightConstants.throwoutDist) &&
        (botpose.getTranslation().getX() > 0) &&
        (botpose.getTranslation().getX() < FieldConstants.fieldLength) &&
        (botpose.getTranslation().getY() > 0) &&
        (botpose.getTranslation().getY() < FieldConstants.fieldWidth)) {
          
          return true;
      } else{
          return false;
      }

  }


  // public Translation2d getTranslationToTag(int tagID) {
  //   if (apriltagPipeline) {
  //     // get botpose from limelight networktables
  //     if (LimelightHelpers.getTV(inputs.name)) {

  //       double[] botPose = LimelightHelpers.getBotPose_wpiBlue(inputs.name);
  //       Pose3d botPose3D = new Pose3d(new Translation3d(botPose[0], botPose[1], botPose[2]),
  //           new Rotation3d(Math.toRadians(botPose[3]), Math.toRadians(botPose[4]), Math.toRadians(botPose[5])));
  //       Pose2d botPose2d = botPose3D.toPose2d();

  //       // get pose of apriltag on field
  //       Optional<Pose3d> aprilTagPose = Constants.AprilTagConstants.layout.getTagPose(tagID);
  //       Logger.recordOutput("Limelight/botpose", botPose3D);

  //       // if we have a tag, calculate our robots distance from it
  //       if (aprilTagPose.isPresent()) {
  //         Logger.recordOutput("Limelight/tagPose2d",
  //             new Pose2d(aprilTagPose.get().getX(), aprilTagPose.get().getY(), new Rotation2d()));
  //         Logger.recordOutput("Limelight/adjustedPose", new Pose2d(aprilTagPose.get().getX() - botPose[0],
  //             aprilTagPose.get().getY() - botPose[1], new Rotation2d()));
  //         return new Translation2d(aprilTagPose.get().getX() - botPose[0], aprilTagPose.get().getY() - botPose[1]);
  //       } else {
  //         return new Translation2d();
  //       }
  //     } else {
  //       return new Translation2d();
  //     }
  //   } else {
  //     return new Translation2d();
  //   }
  // }

  // angle target is from the center
  public Rotation2d getTX() {
    return Rotation2d.fromDegrees(inputs.tx);
  }

  public double getTA() {
    return inputs.ta;
  }

  public void setPriorityID(int tagID, String limelightID) {
    NetworkTableInstance.getDefault().getTable(limelightID).getEntry("priorityid").setDouble(tagID);
  }

}
