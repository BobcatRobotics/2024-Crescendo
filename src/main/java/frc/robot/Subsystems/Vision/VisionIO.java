// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;


import edu.wpi.first.networktables.NetworkTable;

import frc.robot.LimelightHelpers;

/** Vision subsystem hardware interface. */
public interface VisionIO {
  /** The set of loggable inputs for the vision subsystem. */
  
  @AutoLog
  public static class VisionIOInputs{  
      
      LEDMode ledMode = LEDMode.FORCEOFF;
      double pipelineID = 0;
      double pipelineLatency = 0;
      double ta;
      boolean tv;
      double tx;
      double ty;
      double fiducialID;
      double boundingHorizontalPixels;
      
    }
      /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {}

      /** Sets the pipeline number. */
    public default void setLEDS(LEDMode mode) {}

    public default void setPipeline(int index){}


    
  
}