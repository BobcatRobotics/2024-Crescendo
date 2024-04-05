// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Rumble;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rumble extends SubsystemBase {

  // im absolutely not putting the effort to add advantagekit to log when the joystick is rumbling

  XboxController rumbleController;
  Timer timer;
  /** Creates a new Rumble. */
  public Rumble() {
    rumbleController = new XboxController(2);
  }

  /**
   * 
   * @param  [-1,1]
   * @param time seconds
   * @return command that rumbles the joystick
   */
  public void rumble(double intensity, double time){
    timer.reset();
    timer.start();
    rumbleController.setRumble(RumbleType.kBothRumble, intensity);
  }


  @Override
  public void periodic() {

  }
}
