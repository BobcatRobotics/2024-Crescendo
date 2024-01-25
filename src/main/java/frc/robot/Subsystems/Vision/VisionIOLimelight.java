// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class VisionIOLimelight implements VisionIO{
  /** Creates a new VisionIOLimelight. */
  public double captureTimestamp = 0.0;
  public double[] cornerX = new double[] {};
  public double[] cornerY = new double[] {};
  public boolean simpleValid = false;
  public double simpleAngle = 0.0;
  
  private final NetworkTableEntry ledEntry = NetworkTableInstance.getDefault()
    .getTable("limelight").getEntry("ledMode");
  private final NetworkTableEntry pipelineEntry = NetworkTableInstance
      .getDefault().getTable("limelight").getEntry("pipeline");
  private final NetworkTableEntry validEntry =
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
  private final NetworkTableEntry latencyEntry =
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl");
  private final NetworkTableEntry dataEntry = NetworkTableInstance.getDefault()
      .getTable("limelight").getEntry("tcornxy");
  private final NetworkTableEntry simpleAngleEntry =
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");

  public VisionIOLimelight() {
      
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    inputs.captureTimestamp = captureTimestamp;
    inputs.cornerX = cornerX;
    inputs.cornerY = cornerY;
    inputs.simpleValid = simpleValid;
    inputs.simpleAngle = simpleAngle;
  }



  @Override
  public void setPipeline(int pipeline) {
    ledEntry.setInteger(pipeline);
  }

  
}
