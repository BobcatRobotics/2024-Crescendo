// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase{
  /** Creates a new Vision. */
  VisionIO io;
  VisionIOInputsAutoLogged inputs;
  public Vision(VisionIO io) {
    this.io = io;
    this.inputs = new VisionIOInputsAutoLogged();

    
  }

  @Override
  public void periodic(){
  SmartDashboard.putNumberArray("cornerx", inputs.cornerX);
  SmartDashboard.putNumberArray("cornerx", inputs.cornerY);
  }


  /**
   * 
   * @param widthPercent [0,1], percentage of the vertical width of the image that the note is taking up
   * @return distance in meters
   */
  public double distanceFromCameraPercentage(double widthPercent){
    double horizontalLength = Constants.FieldConstants.noteDiameter / widthPercent;
    double cornerFOVAngle = Units.degreesToRadians(90 - (Constants.LimelightConstants.horizontalFOV/2));
    double hypotDist = (horizontalLength/2)*Math.tan(cornerFOVAngle); //distance from note to camera
    double intakeDist = Math.sqrt((hypotDist*hypotDist) - (Constants.LimelightConstants.limelightMountHeight*Constants.LimelightConstants.limelightMountHeight)); //distance to intake
    return intakeDist;
  }
}
