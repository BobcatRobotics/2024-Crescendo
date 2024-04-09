// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto.Align;

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
  private Timer timer = new Timer();
  private Rotation2d angle = new Rotation2d();
  double shootTime;
  private boolean feeding = false;
  private double spivitFudge = 0;

  public AlignAndShoot(Swerve swerve, Spivit spivit, Shooter shooter, Intake intake, double shootTime, double spivitFudge) {
    this.swerve = swerve;
    this.spivit = spivit;
    this.shooter = shooter;
    this.intake = intake;
    this.shootTime = shootTime;
    this.spivitFudge = spivitFudge;

    addRequirements(spivit, shooter, intake, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeed(ShooterConstants.fastShooterRPMSetpoint, ShooterConstants.fastShooterRPMSetpoint);
    finished = false;
    timer.stop();
    timer.reset();
    Logger.recordOutput("Alignment/feeding", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spivit.setAngle(swerve.calcAngleBasedOnHashMap(swerve.getDistanceToSpeakerForSpivit()));
    // spivit.setAngle(swerve.getShootWhileMoveBallistics(spivit.getAngle())[1]);
    
    swerve.drive(new Translation2d(), 0, true, false, true, swerve.getAngleToSpeakerTagAuto().getRadians(), false, 0);//swerve.setRotationTarget(Rotation2d.fromRadians(swerve.getAngleToSpeaker()));
    angle = swerve.getAngleToSpeakerTagAuto();
    // swerve.drive(new Translation2d(), 0, true, false, true, swerve.getAngleToSpeaker());//swerve.setRotationTarget(Rotation2d.fromRadians(swerve.getAngleToSpeaker()));
    // angle = Rotation2d.fromRadians(swerve.getAngleToSpeaker());
    
    Logger.recordOutput("Aligment/spivit", spivit.aligned());
    Logger.recordOutput("Aligment/swerve", swerve.aligned(angle));
    Logger.recordOutput("Aligment/shooter", shooter.aboveSpeed(4500));
    Logger.recordOutput("Alignment/finished", isFinished());
    Logger.recordOutput("feeding", feeding);


    if(spivit.aligned() && swerve.aligned(angle) && shooter.aboveSpeed(4500)){
      timer.start();
      intake.intakeToShooter();
      Logger.recordOutput("Alignment/feeding", true);
      if(timer.hasElapsed(0.1)){
      feeding = true;
      }
      
    }
    if(timer.hasElapsed(shootTime) || (!shooter.aboveSpeed(ShooterConstants.outtookShooterRPMDropThresholdForShootingEarlierInAutos) && feeding)){
      shooter.stop();
      intake.stop();
      spivit.stopMotor();
      swerve.setRotationTarget(null);
      Logger.recordOutput("Alignment/feeding", false);
      finished = true;
      feeding = false;
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    intake.stop();
    timer.stop();
    spivit.stopMotor();
    swerve.setRotationTarget(null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
