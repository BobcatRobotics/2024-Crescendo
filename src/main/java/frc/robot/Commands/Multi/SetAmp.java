// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Multi;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Amp.Amp;
import frc.robot.Subsystems.Spivit.Spivit;

public class SetAmp extends Command {
  private Amp amp;
  private Spivit spivit;
  private boolean deploy;

  /** Creates a new SetAmp. */

  /**
   * 
   * @param amp
   * @param spivit
   * @param deploy true if you want to deploy, false if you want to retract
   */
  public SetAmp(Amp amp, Spivit spivit, boolean deploy) {
    this.amp = amp;
    this.spivit = spivit;
    this.deploy = deploy;
    addRequirements(spivit, amp);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spivit.raiseForAmpMovement();
    SmartDashboard.putBoolean("amp", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("deployed", amp.deployed());
    if (spivit.getAngle() >= ShooterConstants.ampDeploySafeValue - 1.5) {
      if (deploy) {
        amp.deploy();
        if (amp.beyondCrashThreshold()) {
          spivit.raiseForAmpScore();
        }
      } else {
        amp.retract();
        if (amp.retracted()) {
          spivit.stow();
        }
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("amp", true);
    amp.stop();
    spivit.stopMotorFeedforward();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (deploy ? amp.deployed() : amp.retracted()) || DriverStation.isDisabled();
  }
}
