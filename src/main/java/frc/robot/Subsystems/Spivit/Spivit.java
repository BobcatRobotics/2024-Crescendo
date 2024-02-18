// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Spivit;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spivit extends SubsystemBase {
  private final SpivitIO io;
  private final SpivitIOInputsAutoLogged inputs = new SpivitIOInputsAutoLogged();
  
  /** Creates a new Spivit. */
  public Spivit(SpivitIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spivit", inputs);
  }

  /**
   * @param angle IN DEGRES!!!!!!!!!!!!!!>:(
   */
  public void setAngle(double angle){
    io.setAngle(angle);
  }

  public double getAngle(){
    return io.getAngle();
  }
  
  public void stopMotor(){
    io.stopMotor();
  }
  
  public void stopMotorFeedforward(){
    io.stopMotorFeedforward();
  }

  public void setPercent(double percent){
    io.setPercent(percent);
  }


  
}
