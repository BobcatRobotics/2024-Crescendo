// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto.Align;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spivit.Spivit;
import frc.robot.Subsystems.Swerve.Swerve;

public class SmoothieAlignRevOnly extends Command {
  /** Creates a new SmoothieAlignRevOnly. */
  private Swerve swerve;
  private Spivit spivit;
  private Shooter shooter;
  private Intake intake;
  private boolean aligned = false;
  double shootTime = 0;
  double spivitTolerance = 1; //degrees
  double swerveTolerance = 0.1; //radians
  // private boolean feeding = false;
  private boolean finished = false;
  private boolean atSpeed = false;
  private Timer timer = new Timer();
  private boolean haspiece = true;
  private Timer intakeTimer = new Timer();

  public SmoothieAlignRevOnly(Swerve swerve, Spivit spivit, Shooter shooter, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.spivit = spivit;
    this.shooter = shooter;
    this.intake = intake;

    // addRequirements(spivit, shooter, intake);
       addRequirements(spivit, shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      shooter.setSpeed(ShooterConstants.fastShooterRPMSetpoint, ShooterConstants.fastShooterRPMSetpoint);
      timer.reset();
      timer.start();
      intakeTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if (!intake.hasPiece()) {
    //   intake.intakeToShooter();
    // } else {
    //   intake.stop();
    // }

    //swerve - 0 - radians, spivit - 1 - degrees
    double[] ballistics = swerve.getShootWhileMoveBallistics(spivit.getAngle());
    swerve.setRotationTarget(Rotation2d.fromRadians(ballistics[0]));
    spivit.setAngle(ballistics[1]);

    aligned = (Math.abs(ballistics[0] - swerve.getYaw().getRadians())< swerveTolerance) && (Math.abs(ballistics[1] - spivit.getAngle()) < spivitTolerance) && shooter.aboveSpeed(4500);
    
    Logger.recordOutput("Alignment/AutoSmoothie", aligned);

    if(shooter.aboveSpeed(4500) && timer.hasElapsed(0.1)){
      atSpeed = true;
    }

    if(!intake.hasPiece()){
      intakeTimer.reset();
      intakeTimer.start();
      if(intakeTimer.hasElapsed(0.5)){
      haspiece = false;
      }
    }

    if((atSpeed && (!shooter.aboveSpeed(ShooterConstants.outtookShooterRPMDropThresholdForShootingEarlierInAutos))) || (!haspiece)){
      shooter.stop();
      // intake.stop();
      spivit.stopMotor();
      swerve.setRotationTarget(null);
      finished = true;

    }

    // if((!shooter.aboveSpeed(ShooterConstants.outtookShooterRPMDropThresholdForShootingEarlierInAutos) && feeding)){
    //   shooter.stop();
    //   intake.stop();
    //   spivit.stopMotor();
    //   swerve.setRotationTarget(null);
    //   Logger.recordOutput("Alignment/feeding", false);
    //   finished = true;
    //   feeding = false;
    // }



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
