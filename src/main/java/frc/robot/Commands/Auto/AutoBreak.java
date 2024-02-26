// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Spivit.Spivit;

public class AutoBreak extends Command {
  private Spivit spivit;
  private Timer timer = new Timer();

  /** Creates a new AutoBreak. */
  public AutoBreak(Spivit spivit) {
    this.spivit = spivit;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spivit);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spivit.setAngle(ShooterConstants.stow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spivit.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.5);
  }
}
