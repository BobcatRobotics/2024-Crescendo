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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;

public class Vision extends SubsystemBase{
  /** Creates a new Vision. */
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public boolean apriltagPipeline;

  public Vision(VisionIO io) {
    this.io = io;

    io.setLEDS(LEDMode.FORCEOFF);
    // io.setPipeline(Constants.LimelightConstants.apriltagPipelineIndex);
  
  }

  public void setCamMode(CamMode mode){
    io.setCamMode(mode);
  }

  public double getTClass(){
    return inputs.tClass;
  }

  public boolean getTV(){
    return inputs.tv;
  }
  public void setPipeline(String name, int id){
    io.setPipeline(name, id);
  }

  @Override
  public void periodic(){
    io.updateInputs(inputs);
    Logger.processInputs("Limelight" + inputs.name, inputs);

    apriltagPipeline = inputs.pipelineID == 0;

    Logger.recordOutput("note pose/note pose", getNotePose());
    Logger.recordOutput("translation to note", getTranslationToTag((int) inputs.fiducialID));

    
  }

  public double getNoteY(){
    Logger.recordOutput("Limelight"+inputs.name+"/noteY", inputs.distanceToNote*Math.cos(Math.toRadians(90-inputs.tx)));
    return inputs.distanceToNote*Math.cos(Math.toRadians(90-inputs.tx));
  }

  public Pose2d getNotePose(){
    return new Pose2d(inputs.distanceToNote, getNoteY(), Rotation2d.fromDegrees(inputs.tx));
  }

  public Pose2d getBotPose(){
    Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue(inputs.name);
    Logger.recordOutput("Limelight"+inputs.name,pose);
    return pose;
  }

  public double getDistToTag(){
    return LimelightHelpers.getTargetPose_RobotSpace(inputs.name)[0];
  }

  public double getPoseTimestamp(){
    return Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline(inputs.name)/1000.0);
  }

  public Translation2d getTranslationToTag(int tagID){
    if(apriltagPipeline){
      //get botpose from limelight networktables
    if (LimelightHelpers.getTV(inputs.name)){

    double[] botPose = LimelightHelpers.getBotPose_wpiBlue(inputs.name);
    Pose3d botPose3D = new Pose3d(new Translation3d(botPose[0], botPose[1], botPose[2]), new Rotation3d(Math.toRadians(botPose[3]), Math.toRadians(botPose[4]), Math.toRadians(botPose[5])));
    Pose2d botPose2d = botPose3D.toPose2d();
    
    //get pose of apriltag on field
    Optional<Pose3d> aprilTagPose = Constants.AprilTagConstants.layout.getTagPose(tagID);
    Logger.recordOutput("Limelight/botpose", botPose3D);
    
    //if we have a tag, calculate our robots distance from it
    if (aprilTagPose.isPresent()){
      Logger.recordOutput("Limelight/tagPose2d", new Pose2d(aprilTagPose.get().getX(), aprilTagPose.get().getY(), new Rotation2d()));
      Logger.recordOutput("Limelight/adjustedPose", new Pose2d(aprilTagPose.get().getX() - botPose[0], aprilTagPose.get().getY() - botPose[1], new Rotation2d()));
      return new Translation2d(aprilTagPose.get().getX() - botPose[0], aprilTagPose.get().getY() - botPose[1]);
    }else{
      return new Translation2d();
    }
    }else {
      return new Translation2d();
    }
    }else{
      return new Translation2d();
    }
  }

  //angle target is from the center
  public Rotation2d getTX(){
    return Rotation2d.fromDegrees(inputs.tx);
  }











 
}
