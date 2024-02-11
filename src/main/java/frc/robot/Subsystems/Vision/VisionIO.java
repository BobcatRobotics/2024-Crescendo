// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;

import frc.robot.LimelightHelpers;

/** Vision subsystem hardware interface. */
public interface VisionIO {
  /** The set of loggable inputs for the vision subsystem. */
  
  @AutoLog
  public static class VisionIOInputs{  
    //intake
      // public LEDMode ledModeIntake = LEDMode.FORCEOFF;
      // public double pipelineIDIntake = 0;
      // public double pipelineLatencyIntake = 0;
      // public double taIntake;
      // public boolean tvIntake;
      // public double txIntake;
      // public double tyIntake;
      // public double fiducialIDIntake;
      // public double boundingHorizontalPixelsIntake;
      // public double distanceToNoteIntake;
      // public double rawDistanceToNoteIntake;
      // public double tClassIntake;

      // //shooterleft
      // public LEDMode ledModeShooterleft = LEDMode.FORCEOFF;
      // public double pipelineIDShooterleft = 0;
      // public double pipelineLatencyShooterleft = 0;
      // public double taShooterleft;
      // public boolean tvShooterleft;
      // public double txShooterleft;
      // public double tyShooterleft;
      // public double fiducialIDShooterleft;

      // //shooterright
      // public LEDMode ledModeShooterright = LEDMode.FORCEOFF;
      // public double pipelineIDShooterright = 0;
      // public double pipelineLatencyShooterright = 0;
      // public double taShooterright;
      // public boolean tvShooterright;
      // public double txShooterright;
      // public double tyShooterright;
      // public double fiducialIDShooterright;

      //generic
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

  /**
   * 
   * @param widthPercent [0,1], percentage of the vertical width of the image that the note is taking up
   * @return distance in meters
   */
  public default double distanceFromCameraPercentage(double widthPercent){
   return 0.0;
  }
    
  
}