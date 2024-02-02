// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase{
  /** Creates a new Vision. */
  VisionIO io;
  VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  

  public Vision(VisionIO io) {
    this.io = io;
    
    io.setLEDS(LEDMode.FORCEOFF);
    io.setPipeline(Constants.LimelightConstants.detectorPiplineIndex);
    
  
  }

  public double getTClass(){
    return inputs.tClass;
  }

  @Override
  public void periodic(){
    io.updateInputs(inputs);

    Logger.recordOutput("note pose/note pose", getNotePose());
  }

  public double getNoteY(){
    
    Logger.recordOutput("Limemight/noteY", inputs.distanceToNote*Math.cos(Math.toRadians(90-inputs.tx)));
    return inputs.distanceToNote*Math.cos(Math.toRadians(90-inputs.tx));
    
  }

  public Pose2d getNotePose(){
    return new Pose2d(inputs.distanceToNote, getNoteY(), Rotation2d.fromDegrees(inputs.tx));
  }






 
}
