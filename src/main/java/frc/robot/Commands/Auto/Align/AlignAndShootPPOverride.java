// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto.Align;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spivit.Spivit;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Util.BobcatUtil;

public class AlignAndShootPPOverride extends Command {
  /** Creates a new AlignToShooter. */
  private Swerve swerve;
  private Spivit spivit;
  private Shooter shooter;
  private Intake intake;
  private boolean finished = false;
  private Timer timer = new Timer();
  double shootTime;
  double spivitOffset = 0;

  public AlignAndShootPPOverride(Swerve swerve, Spivit spivit, Shooter shooter, Intake intake, double shootTime, double spivitOffset) {
    this.swerve = swerve;
    this.spivit = spivit;
    this.shooter = shooter;
    this.intake = intake;
    this.shootTime = shootTime;
    this.spivitOffset = spivitOffset;

    addRequirements(spivit, shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeed(ShooterConstants.fastShooterRPMSetpoint, ShooterConstants.fastShooterRPMSetpoint);
    finished = false;
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spivit.setAngle(swerve.calcAngleBasedOnRealRegression() + spivitOffset);
    
    if (BobcatUtil.isRed()) {
      swerve.setRotationTarget(Rotation2d.fromRadians(swerve.getShootWhileMoveBallistics()[0]));
    } else {
      swerve.setRotationTarget(Rotation2d.fromRadians(swerve.getShootWhileMoveBallistics()[0]).rotateBy(Rotation2d.fromDegrees(180)));
    }
    Logger.recordOutput("Aligment/spivit", spivit.aligned());
    Logger.recordOutput("Aligment/swerve", swerve.aligned());
    Logger.recordOutput("Aligment/shooter", shooter.aboveSpeed(4500));
    Logger.recordOutput("Alignment/finished", isFinished());
    

    if(spivit.aligned() && swerve.aligned() && shooter.aboveSpeed(4500)){
      timer.start();
      intake.intakeToShooter();
    }
    if(timer.hasElapsed(shootTime)){
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
