// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;

public class RevToRPM extends Command {
  
  private Shooter shooter;
  private double rpm;
  private double threshold;
  boolean finished = false;

  /** 
   * sets the shooter to a specific RPM, command ends when threshold is reached.
   * this does NOT stop the shooter on end
   */
  public RevToRPM(Shooter shooter, double rpm, double threshold) {
    this.shooter = shooter;
    this.rpm = rpm;
    this.threshold = threshold;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    shooter.setSpeed(rpm, rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.aboveSpeed(threshold)){
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
