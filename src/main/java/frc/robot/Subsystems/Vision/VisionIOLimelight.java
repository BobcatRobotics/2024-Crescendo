// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class VisionIOLimelight implements VisionIO{
  /** Creates a new VisionIOLimelight. */
  LEDMode currentLedMode = LEDMode.FORCEOFF;
  LinearFilter distanceFilter = LinearFilter.singlePoleIIR(Constants.LimelightConstants.filterTimeConstant, Constants.loopPeriodSecs);

  public VisionIOLimelight() {

  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
  
    inputs.ledMode = currentLedMode;
    inputs.pipelineID = LimelightHelpers.getCurrentPipelineIndex(null);
    inputs.pipelineLatency = LimelightHelpers.getLatency_Pipeline(null);
    inputs.ta = LimelightHelpers.getTA(null);
    inputs.tv = LimelightHelpers.getTV(null);
    inputs.tx = LimelightHelpers.getTX(null);
    inputs.ty = LimelightHelpers.getTY(null);
    inputs.fiducialID = LimelightHelpers.getFiducialID(null);
    inputs.boundingHorizontalPixels = LimelightHelpers.getLimelightNTDouble(null, "thor");
    inputs.distanceToNote = distanceFilter.calculate(distanceFromCameraPercentage(pixlesToPercent(inputs.boundingHorizontalPixels)));
    inputs.rawDistanceToNote = distanceFromCameraPercentage(pixlesToPercent(inputs.boundingHorizontalPixels));
    inputs.rawXdistToNote = ((180*Constants.FieldConstants.noteDiameter)/(63.3*Math.PI)) * (1/pixlesToPercent(inputs.boundingHorizontalPixels));
    inputs.XdistToNote = distanceFilter.calculate(inputs.XdistToNote);
    Logger.recordOutput("Limelight/rawDistance", inputs.rawDistanceToNote);
    Logger.recordOutput("Limelight/filteredDistance", inputs.distanceToNote);
    Logger.recordOutput("Limelight/rawXDist", inputs.rawXdistToNote);
    Logger.recordOutput("Limelight/filteredXDist", inputs.XdistToNote);

  }



  @Override
  public void setLEDS(LEDMode mode) {
    switch (mode) {
      case FORCEBLINK:
        LimelightHelpers.setLEDMode_ForceBlink(null);
        currentLedMode = LEDMode.FORCEBLINK;
        break;
      case FORCEOFF:
        LimelightHelpers.setLEDMode_ForceOff(null);
        currentLedMode = LEDMode.FORCEOFF;
      case FORCEON:
        LimelightHelpers.setLEDMode_ForceOn(null);
        currentLedMode = LEDMode.FORCEON;
      case PIPELINECONTROL:
        LimelightHelpers.setLEDMode_PipelineControl(null);
        currentLedMode = LEDMode.PIPELINECONTROL;
      default:
        LimelightHelpers.setLEDMode_ForceOff(null);
        currentLedMode = LEDMode.FORCEOFF;
        break;
    }
  }

  @Override
  public void setPipeline(int index){    
    LimelightHelpers.setPipelineIndex(null, index);
  }

  public double pixlesToPercent(double pixels){
    Logger.recordOutput("Limelight/horPercent", pixels/Constants.LimelightConstants.horPixles);  
    return pixels/Constants.LimelightConstants.horPixles;
  }

  /**
   * 
   * @param widthPercent [0,1], percentage of the vertical width of the image that the note is taking up
   * @return distance in meters
   */
  public double distanceFromCameraPercentage(double widthPercent){
    // double horizontalLength = Constants.FieldConstants.noteDiameter / widthPercent;
    // double cornerFOVAngle = Units.degreesToRadians(90 - (Constants.LimelightConstants.horizontalFOV/2));
    // double hypotDist = (horizontalLength/2)*Math.tan(cornerFOVAngle); //distance from note to camera
    double hypotDist = ((180*Constants.FieldConstants.noteDiameter)/(63.3*Math.PI)) * (1/widthPercent);
    double intakeDist = Math.sqrt((hypotDist*hypotDist) - (Constants.LimelightConstants.limelightMountHeight*Constants.LimelightConstants.limelightMountHeight)); //distance to intake
    
    return intakeDist;
  }
}
