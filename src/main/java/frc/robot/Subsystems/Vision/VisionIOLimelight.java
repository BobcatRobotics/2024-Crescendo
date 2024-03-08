// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.numbers.N3;
import frc.lib.util.limelightConstants;
import frc.robot.Constants;

import frc.robot.LimelightHelpersFast;

public class VisionIOLimelight implements VisionIO{
  /** Creates a new VisionIOLimelight. */
    LEDMode currentLedMode = LEDMode.FORCEOFF;
    CamMode currentCamMode = CamMode.VISION;
    public final String name;
    public final double verticalFOV;
    public final double horizontalFOV;
    public final double limelightMountHeight;
    public final int detectorPiplineIndex; 
    public final int apriltagPipelineIndex;
    public final int horPixels;
    public final double filterTimeConstant; // in seconds, inputs occuring over a time period significantly shorter than this will be thrown out
    public final Vector<N3> visionMeasurementStdDevs;
    public final int movingAverageNumTaps;
    public final LinearFilter distanceFilter;


  public VisionIOLimelight(limelightConstants limelightConstants) {
    name = limelightConstants.name;
    verticalFOV = limelightConstants.verticalFOV;
    horizontalFOV = limelightConstants.horizontalFOV;
    limelightMountHeight=limelightConstants.limelightMountHeight;
    detectorPiplineIndex=limelightConstants.detectorPiplineIndex;
    apriltagPipelineIndex=limelightConstants.apriltagPipelineIndex;
    horPixels=limelightConstants.horPixels;
    filterTimeConstant=limelightConstants.filterTimeConstant;
    visionMeasurementStdDevs=limelightConstants.visionMeasurementStdDevs;
    movingAverageNumTaps=limelightConstants.movingAverageNumTaps;
    distanceFilter = LinearFilter.movingAverage(movingAverageNumTaps);


  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.ledMode = currentLedMode;
    inputs.camMode = currentCamMode;
    inputs.pipelineID = LimelightHelpersFast.getCurrentPipelineIndex(name);
    inputs.pipelineLatency = LimelightHelpersFast.getLatency_Pipeline(name);
    inputs.ta = LimelightHelpersFast.getTA(name);
    inputs.tv = LimelightHelpersFast.getTV(name);
    inputs.tx = LimelightHelpersFast.getTX(name);
    inputs.ty = LimelightHelpersFast.getTY(name);
    inputs.fiducialID = LimelightHelpersFast.getFiducialID(name);
    inputs.boundingHorizontalPixels = LimelightHelpersFast.getLimelightNTDouble(name, "thor");
    inputs.distanceToNote = distanceFilter.calculate(distanceFromCameraPercentage(inputs.boundingHorizontalPixels));
    inputs.rawDistanceToNote = distanceFromCameraPercentage(inputs.boundingHorizontalPixels);
    inputs.tClass=LimelightHelpersFast.getNeuralClassID(name);
    inputs.name=name;
  }



  @Override
  public void setLEDS(LEDMode mode) {
    switch (mode) {
      case FORCEBLINK:
        LimelightHelpersFast.setLEDMode_ForceBlink(name);
        currentLedMode = LEDMode.FORCEBLINK;
        break;
      case FORCEOFF:
        LimelightHelpersFast.setLEDMode_ForceOff(name);
        currentLedMode = LEDMode.FORCEOFF;
      case FORCEON:
        LimelightHelpersFast.setLEDMode_ForceOn(name);
        currentLedMode = LEDMode.FORCEON;
      case PIPELINECONTROL:
        LimelightHelpersFast.setLEDMode_PipelineControl(name);
        currentLedMode = LEDMode.PIPELINECONTROL;
      default:
        LimelightHelpersFast.setLEDMode_ForceOff(name);
        currentLedMode = LEDMode.FORCEOFF;
        break;
    }
  }

  @Override
  public void setCamMode(CamMode mode){
    switch (mode){
      case DRIVERCAM:
      LimelightHelpersFast.setCameraMode_Driver(name);
      currentCamMode = CamMode.DRIVERCAM;
      case VISION:
      LimelightHelpersFast.setCameraMode_Processor(name);
      currentCamMode = CamMode.VISION;
    }
  }

  @Override
  public void setPipeline(String limelight, int index){    
    LimelightHelpersFast.setPipelineIndex(limelight, index);
  }

  public double pixlesToPercent(double pixels){
    Logger.recordOutput("Limelight/horPercent", pixels/horPixels);
    return pixels/horPixels;
  }

  /**
   * 
   * @param widthPercent [0,1], percentage of the vertical width of the image that the note is taking up
   * @return distance in meters
   */
  public double distanceFromCameraPercentage(double widthPercent){
    
    if (LimelightHelpersFast.getTV(name)){
    widthPercent = pixlesToPercent(widthPercent);
    // double horizontalLength = Constants.FieldConstants.noteDiameter / widthPercent;
    // double cornerFOVAngle = Units.degreesToRadians(90 - (Constants.LimelightConstants.horizontalFOV/2));
    // double hypotDist = (horizontalLength/2)*Math.tan(cornerFOVAngle); //distance from note to camera
    double hypotDist = ((180*Constants.FieldConstants.noteDiameter)/(63.3*Math.PI)) * (1/widthPercent);
    double intakeDist = Math.sqrt((hypotDist*hypotDist) - (limelightMountHeight*limelightMountHeight)); //distance to intake
    
    return intakeDist;
    }else{
      return 0;
    }
  }
}
