// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;




import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spivit.Spivit;
import frc.robot.Subsystems.Swerve.Swerve;

public class SubwooferShot extends Command {
  private Swerve swerve;
  private Spivit spivit;
  private Shooter shooter;
  private double shooterRPM;
  private boolean upToSpeed = false;
  private Timer timer = new Timer();
  private boolean finished = false;
  /**
   * 
   * @param end ends the command, not sure if this is needed
   */
  public SubwooferShot(Swerve swerve, Spivit spivit, Shooter shooter, double shooterRPM) {
    this.swerve = swerve;
    this.spivit = spivit;
    this.shooter = shooter;
    this.shooterRPM = shooterRPM;

    addRequirements(spivit, shooter);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeed(shooterRPM, shooterRPM);
    spivit.setAngle(swerve.calcAngleBasedOnHashMap());
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spivit.setAngle(swerve.calcAngleBasedOnHashMap());
    if(shooter.aboveSpeed(3700)){
      upToSpeed = true;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spivit.stopMotor();
    upToSpeed = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return upToSpeed;
  }
}
