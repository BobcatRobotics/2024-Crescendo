// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto.Align;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants.intake;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spivit.Spivit;
import frc.robot.Subsystems.Swerve.Swerve;

public class SmoothieShootOnly extends Command {
  /** Creates a new SmoothieShootOnly. */
  private Intake intake;
  private boolean finished = false;
  private double shootTime;
  private Spivit spivit;
  private Swerve swerve;
  private boolean aligned;
  private double spivitTolerance = 1; // degrees
  private double swerveTolerance = 0.1; // radians
  private Shooter shooter;
  private Timer timer = new Timer();
  private boolean feeding = false;

  public SmoothieShootOnly(Intake intake, double shootTime, Spivit spivit, Swerve swerve, Shooter shooter) {
    this.intake = intake;
    this.shootTime = shootTime;
    this.spivit = spivit;
    this.swerve = swerve;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] ballistics = swerve.getShootWhileMoveBallistics(spivit.getAngle());

    aligned = (Math.abs(ballistics[0] - swerve.getYaw().getRadians()) < swerveTolerance)
        && (Math.abs(ballistics[1] - spivit.getAngle()) < spivitTolerance) && shooter.aboveSpeed(4500);
    if (aligned) {
      timer.start();
      intake.intakeToShooter();
      Logger.recordOutput("Alignment/feeding", true);
      if (timer.hasElapsed(0.1)) {
        feeding = true;
      }
    }

    if (timer.hasElapsed(shootTime)
        || (!shooter.aboveSpeed(ShooterConstants.outtookShooterRPMDropThresholdForShootingEarlierInAutos) && feeding)) {
      intake.stop();
      Logger.recordOutput("Alignment/feeding", false);
      finished = true;
      feeding = false;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
