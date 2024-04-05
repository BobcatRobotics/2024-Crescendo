// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;




import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spivit.Spivit;

public class PreloadOnly extends Command {
  private Spivit spivit;
  private Shooter shooter;
  private double shooterRPM;
  private Intake intake;
  /**
   * 
   * @param end ends the command, not sure if this is needed
   */


  //ABSOLUTELY DO NOT USE THIS >:(
  public PreloadOnly(Spivit spivit, Shooter shooter, double shooterRPM, Intake intake) {
    this.spivit = spivit;
    this.shooter = shooter;
    this.shooterRPM = shooterRPM;
    this.intake = intake;
    addRequirements(spivit, shooter);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeed(shooterRPM, shooterRPM);
    spivit.setAngle(ShooterConstants.subwooferShot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.aboveSpeed(3700)){
      intake.intakeToShooter();
    }
    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spivit.stopMotorFeedforward();
    shooter.stop();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
