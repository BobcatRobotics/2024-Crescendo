// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Spivit.Spivit;

public class ReleaseHook extends Command {
  private Spivit spivit;
  private Timer timer = new Timer();
  private boolean released = false;

  /** Creates a new ReleaseHook. */
  public ReleaseHook(Spivit spivit) {
    this.spivit = spivit;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spivit);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    released = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (spivit.getAngle() < ShooterConstants.releasehookSetpoint) {
      spivit.setPercent(0.35);
    } else if (!timer.hasElapsed(0.5) && released) {
      spivit.stopMotorFeedforward();
    } else if (released && timer.hasElapsed(0.5)) {
      spivit.setPercent(-0.1);
    } else {
      timer.start();
      released = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spivit.stopMotorFeedforward();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1);
  }
}
