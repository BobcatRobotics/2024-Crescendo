// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto.Align;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spivit.Spivit;
import frc.robot.Subsystems.Swerve.Swerve;

public class SmoothieAlignDontShoot extends Command {

  private Swerve swerve;
  private Spivit spivit;
  private Shooter shooter;
  private Intake intake;
  private boolean aligned = false;
  // double shootTime = 0;
  double spivitTolerance = 0.5; //degrees
  double swerveTolerance = Math.toRadians(2.5); //radians
  private Timer timer = new Timer();
  private boolean feeding = false;
  private boolean finished = false;


  /** Creates a new SmoothieAlignDontShoot. */
  public SmoothieAlignDontShoot(Swerve swerve, Spivit spivit, Shooter shooter, Intake intake) {
    this.swerve = swerve;
    this.spivit = spivit;
    this.shooter = shooter;
    this.intake = intake;

    addRequirements(spivit, shooter);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  shooter.setSpeed(ShooterConstants.fastShooterRPMSetpoint, ShooterConstants.fastShooterRPMSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] ballistics = swerve.getShootWhileMoveBallistics(spivit.getAngle());
    swerve.setRotationTarget(Rotation2d.fromRadians(ballistics[0]));
    spivit.setAngle(ballistics[1]);

    aligned = (Math.abs(ballistics[0] - swerve.getYaw().getRadians())< swerveTolerance) && (Math.abs(ballistics[1] - spivit.getAngle()) < spivitTolerance) && shooter.aboveSpeed(4500);
    
    if(aligned){
      // timer.start();
      // intake.intakeToShooter();
      Logger.recordOutput("Alignment/feeding", true);
      // if(timer.hasElapsed(0.1)){
      // feeding = true;
      swerve.SetSmoothieAutoIsAligned(aligned);

      }

      // if(!intake.hasPiece()){
      //   intake.intakeToShooter();
      // }
      // else{
      //   intake.stop();
      // }



    }

    
    // if(timer.hasElapsed(shootTime) || (!shooter.aboveSpeed(ShooterConstants.outtookShooterRPMDropThresholdForShootingEarlierInAutos) && feeding)){
    //   shooter.stop();
    //   intake.stop();
    //   spivit.stopMotor();
    //   swerve.setRotationTarget(null);
    //   Logger.recordOutput("Alignment/feeding", false);
    //   finished = true;
    //   feeding = false;
    // }
  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setRotationTarget(null);
      shooter.stop();
      spivit.stopMotor();
      swerve.setRotationTarget(null);
      Logger.recordOutput("Alignment/feeding", false);
      finished = true;
      feeding = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
