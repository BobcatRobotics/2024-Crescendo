// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto.Align;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;

public class StopSwervePPOverride extends Command {
  private Swerve swerve;
  private boolean finished = false;
  /** Creates a new StopSwerveOverride. */
  public StopSwervePPOverride(Swerve swerve) {
    this.swerve=swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.setRotationTarget(null);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finished=true;
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
