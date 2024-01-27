// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;


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

  @Override
  public void periodic(){
    io.updateInputs(inputs); 
  }





 
}
