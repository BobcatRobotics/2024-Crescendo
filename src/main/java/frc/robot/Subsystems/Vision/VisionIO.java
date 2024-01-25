// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Vision subsystem hardware interface. */
public interface VisionIO {
  /** The set of loggable inputs for the vision subsystem. */
  
  @AutoLog
  public static class VisionIOInputs implements LoggableInputs{  
    public double captureTimestamp = 0.0;
    public double[] cornerX = new double[] {};
    public double[] cornerY = new double[] {};
    public boolean simpleValid = false;
    public double simpleAngle = 0.0;
  

    public void toLog(LogTable table) {
      table.put("CaptureTimestamp", captureTimestamp);
      table.put("CornerX", cornerX);
      table.put("CornerY", cornerY);
      table.put("SimpleValid", simpleValid);
      table.put("SimpleAngle", simpleAngle);
    }

    public void fromLog(LogTable table) {
      captureTimestamp = table.get("CaptureTimestamp", captureTimestamp);
      cornerX = table.get("CornerX", cornerX);
      cornerY = table.get("CornerY", cornerY);
      simpleValid = table.get("SimpleValid", simpleValid);
      simpleAngle = table.get("SimpleAngle", simpleAngle);
    }
  }
      /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {}

      /** Sets the pipeline number. */
    public default void setPipeline(int pipeline) {}
  
}