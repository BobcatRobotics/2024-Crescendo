// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spivit.Spivit;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.ShootingParameters;

public class SmoothieAuto extends Command {
  /** Creates a new AlignToShooter. */
  private Swerve swerve;
  private Spivit spivit;
  private Shooter shooter;
  private Intake intake;
  private Timer timer = new Timer();
  private boolean feeding = false;
  private Rotation2d alignSetpoint;
  
  public double shootTime = 1.5;
  public boolean intaking = false;
  public boolean aligningAndReving = true;
  public boolean shouldShoot = true;
  public boolean preload = true;

  public SmoothieAuto(Swerve swerve, Spivit spivit, Shooter shooter, Intake intake) {
    this.swerve = swerve;
    this.spivit = spivit;
    this.shooter = shooter;
    this.intake = intake;

    addRequirements(spivit, shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intaking) {
        shouldShoot = false;
        if (intake.hasPiece()) {
            intake.stop();
            intaking = false;
        } else {
            intake.intakeToShooter();
        }
    }
    
    if (aligningAndReving) {
        ShootingParameters params = swerve.getCheesyPoofsShootOnTheFly(true, spivit.getAngle());
        spivit.setAngle(ShooterConstants.spivitAngles.get(params.effective_range_m));
        swerve.setRotationTarget(Rotation2d.fromDegrees(params.effective_yaw_angle_deg));
        alignSetpoint = Rotation2d.fromDegrees(params.effective_yaw_angle_deg);
        shooter.setSpeed(ShooterConstants.fastShooterRPMSetpoint, ShooterConstants.fastShooterRPMSetpoint);
    } else {
        swerve.setRotationTarget(null);
        spivit.stopMotorFeedforward();
        shooter.stop();
    }

    if (shouldShoot) {
        if (spivit.aligned() && Math.abs(swerve.getYaw().getDegrees() - alignSetpoint.getDegrees()) < 2.5 && shooter.aboveSpeed(4500)) {
            timer.start();
            intake.intakeToShooter();
            if (timer.hasElapsed(0.1)) {
                feeding = true;
            }
        } else if (preload && spivit.aligned() && Math.abs(swerve.getYaw().getDegrees() - alignSetpoint.getDegrees()) < 2.5 && shooter.aboveSpeed(3500)) {
            timer.start();
            intake.intakeToShooter();
            if (timer.hasElapsed(0.1)) {
                feeding = true;
            }
        }

        if (feeding && (timer.hasElapsed(shootTime) || (!shooter.aboveSpeed(ShooterConstants.outtookShooterRPMDropThresholdForShootingEarlierInAutos)))) {
            shouldShoot = false;
            aligningAndReving = false;
            feeding = false;
            intake.stop();
            timer.stop();
            timer.reset();
            preload = false;
        }
    } else {
        timer.stop();
        timer.reset();
        feeding = false;
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
    return false;
  }
}
