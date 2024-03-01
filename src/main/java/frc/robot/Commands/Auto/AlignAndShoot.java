// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spivit.Spivit;
import frc.robot.Subsystems.Swerve.Swerve;

public class AlignAndShoot extends Command {
  /** Creates a new AlignToShooter. */
  private Swerve swerve;
  private Spivit spivit;
  private Shooter shooter;
  private Intake intake;
  private boolean finished = false;
  public AlignAndShoot(Swerve swerve, Spivit spivit, Shooter shooter, Intake intake) {
    this.swerve = swerve;
    this.spivit = spivit;
    this.shooter = shooter;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeed(ShooterConstants.fastShooterRPMSetpoint, ShooterConstants.fastShooterRPMSetpoint);
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Timer timer = new Timer();
    timer.reset();
    spivit.setAngle(swerve.calcAngleBasedOnRealRegression());
    
    
    swerve.setRotationTarget(Rotation2d.fromRadians(swerve.getAngleToSpeaker()).rotateBy(Rotation2d.fromDegrees(180)));
    Logger.recordOutput("Aligment/spivit", spivit.aligned());
    Logger.recordOutput("Aligment/swerve", swerve.aligned());
    Logger.recordOutput("Aligment/shooter", shooter.aboveSpeed(3000));
    Logger.recordOutput("Alignment/finished", isFinished());

    if(spivit.aligned() && swerve.aligned() && shooter.aboveSpeed(3000)){
      timer.start();
      intake.intakeToShooter();
    }
    if(timer.hasElapsed(1)){
      shooter.stop();
      intake.stop();
      spivit.stopMotor();
      swerve.setRotationTarget(null);
      finished = true;
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    intake.stop();
    spivit.stopMotor();
    swerve.setRotationTarget(null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
