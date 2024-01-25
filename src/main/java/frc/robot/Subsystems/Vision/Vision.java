// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Vision.VisionIO;
public class Vision extends Command {
  /** Creates a new Vision. */
  VisionIO io;
  VisionIOInputsAutoLogged inputs;
  public Vision(VisionIO io) {
    this.io = io;
    this.inputs = new VisionIOInputsAutoLogged();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(inputs.cornerX);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
