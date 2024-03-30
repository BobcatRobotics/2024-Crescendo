// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.CANdle;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdle extends SubsystemBase {
  private final CANdleIO io;
  private final CANdleIOInputsAutoLogged inputs = new CANdleIOInputsAutoLogged();
  private double seconds = 1;
  private Timer timer = new Timer();

  /** Creates a new CANdle. */
  public CANdle(CANdleIO candle) {
    io = candle;
    timer.reset();
    
  }

  /**
   * THIS WILL NOT AUTOMATICALLY TURN OFF, it will persist untill you set it again
   * @param state the animation to play
   */
  public void setLEDs(CANdleState state){
    timer.stop();
    timer.reset();
    io.setLEDs(state);
  }

  /**
   * 
   * @param state the animation to play
   * @param seconds duration to play
   */
  public void setLEDs(CANdleState state, double seconds){
    setLEDs(state);
    this.seconds = seconds;
    timer.reset();
    timer.start();
  }

  public CANdleState getState(){
    return inputs.state;
  }

  @Override
  public void periodic() {

    if(timer.hasElapsed(seconds)){
      setLEDs(CANdleState.OFF);
      timer.stop();
      timer.reset();
      seconds = 1;
    }

    io.updateInputs(inputs);
    Logger.processInputs("CANdle", inputs);
  }
  
}
