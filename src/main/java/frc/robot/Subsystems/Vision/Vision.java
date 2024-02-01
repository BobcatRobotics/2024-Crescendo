// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
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
  }

  public double getNoteY(){
    Logger.recordOutput("Limelight/noteY", inputs.distanceToNote*Math.sin(Math.toRadians(inputs.tx)));
    return inputs.distanceToNote*Math.sin(Math.toRadians(inputs.tx));
    
  }

  public double getRotation(){
    Logger.recordOutput("Limelight/noteRot", Math.toRadians(inputs.tx));
    return Math.toRadians(inputs.tx);
  }

  public Translation2d getNoteTranslation(PIDController xpid, PIDController ypid){
    return new Translation2d(xpid.calculate(inputs.distanceToNote, 0), ypid.calculate(getNoteY(), 0));
  }






 
}
