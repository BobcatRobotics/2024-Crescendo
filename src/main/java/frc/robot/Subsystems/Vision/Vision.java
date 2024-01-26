// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase{
  /** Creates a new Vision. */
  VisionIO io;
  VisionIOInputsAutoLogged inputs;
  public Vision(VisionIO io) {
    this.io = io;
    this.inputs = new VisionIOInputsAutoLogged();
    
    io.setLEDS(LEDMode.FORCEOFF);
    io.setPipeline(Constants.LimelightConstants.detectorPiplineIndex);
  }

  @Override
  public void periodic(){
    io.updateInputs(inputs);
    
    Logger.recordOutput("Limelight/estimatedDistance", distanceFromCameraPercentage(pixlesToPercent(inputs.boundingHorizontalPixels)));
  }

  
  public double pixlesToPercent(double pixels){
    Logger.recordOutput("limelight/horPercent", pixels/Constants.LimelightConstants.horPixles);
    return pixels/Constants.LimelightConstants.horPixles;
  }

  /**
   * 
   * @param widthPercent [0,1], percentage of the vertical width of the image that the note is taking up
   * @return distance in meters
   */
  @AutoLogOutput
  public double distanceFromCameraPercentage(double widthPercent){
    // double horizontalLength = Constants.FieldConstants.noteDiameter / widthPercent;
    // double cornerFOVAngle = Units.degreesToRadians(90 - (Constants.LimelightConstants.horizontalFOV/2));
    // double hypotDist = (horizontalLength/2)*Math.tan(cornerFOVAngle); //distance from note to camera
    double hypotDist = ((180*Constants.FieldConstants.noteDiameter)/(63.3*Math.PI)) * (1/widthPercent);
    double intakeDist = Math.sqrt((hypotDist*hypotDist) - (Constants.LimelightConstants.limelightMountHeight*Constants.LimelightConstants.limelightMountHeight)); //distance to intake
    return intakeDist;
  }


 
}
