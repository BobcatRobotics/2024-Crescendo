// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Multi;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Amp.Amp;
import frc.robot.Subsystems.Spivit.Spivit;

public class SetAmp extends Command {
  private Amp amp;
  private Spivit spivit;

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
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    if(spivit.getAngle() < ShooterConstants.ampDeploySafeValue){
      spivit.setAngle(ShooterConstants.ampDeploySafeValue);
    }else{
      amp.setPos(AmpConstants.deployValue);
    }

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
