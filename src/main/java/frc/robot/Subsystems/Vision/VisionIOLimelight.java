// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;


import frc.robot.LimelightHelpers;

public class VisionIOLimelight implements VisionIO{
  /** Creates a new VisionIOLimelight. */
  LEDMode currentLedMode = LEDMode.FORCEOFF;

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

  
}
