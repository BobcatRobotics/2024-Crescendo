// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;



/** Vision subsystem hardware interface. */
public interface VisionIO {
  /** The set of loggable inputs for the vision subsystem. */
  
  @AutoLog
  public static class VisionIOInputs{  
      public LEDMode ledMode = LEDMode.FORCEOFF;
      public double pipelineID = 0;
      public double pipelineLatency = 0;
      public double ta;
      public boolean tv;
      public double tx;
      public double ty;
      public double fiducialID;
      public double boundingHorizontalPixels;
      public double distanceToNote;
      public double rawDistanceToNote;
      public double tClass;
      public String name;
      public CamMode camMode = CamMode.VISION;
    }
      /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {}

      /** Sets the pipeline number. */
    public default void setLEDS(LEDMode mode) {}

    public default void setPipeline(String limelight, int index){}

    public default double pixlesToPercent(double pixels){
      return 0.0;
    }
    public default double getTClass(){
      return 0.0;
    }

    public default void setCamMode(CamMode mode){}

  /**
   * 
   * @param widthPercent [0,1], percentage of the vertical width of the image that the note is taking up
   * @return distance in meters
   */
  public default double distanceFromCameraPercentage(double widthPercent){
   return 0.0;
  }
    
  
}