// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spivit.Spivit;
import frc.robot.Subsystems.Swerve.Swerve;

public class ContinouslyAlignAndShoot extends Command {
  private Swerve swerve;
  private Spivit spivit;
  private Shooter shooter;
  private Intake intake;
  private BooleanSupplier finished;
  private double shooterRPM;

  /**
   * 
   * @param end ends the command, not sure if this is needed
   */
  public ContinouslyAlignAndShoot(Swerve swerve, Spivit spivit, Shooter shooter, Intake intake, BooleanSupplier end, double shooterRPM) {
    this.swerve = swerve;
    this.spivit = spivit;
    this.shooter = shooter;
    this.intake = intake;
    this.finished = end;
    this.shooterRPM = shooterRPM;

    addRequirements(spivit, shooter, intake);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeed(shooterRPM, shooterRPM);
    spivit.setAngle(swerve.calcAngleBasedOnRealRegression());
    intake.intakeToShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //need to rotate by 180 deg to account for pathplanner strangeness
    swerve.setRotationTarget(Rotation2d.fromRadians(swerve.getAngleToSpeaker()).rotateBy(Rotation2d.fromDegrees(180)));
    //shooter.setSpeed(shooterRPM, shooterRPM);
    spivit.setAngle(swerve.calcAngleBasedOnRealRegression()-2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setRotationTarget(null); //stops overriding the pathplanner rotation
    shooter.stop();
    intake.stop();
    spivit.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished.getAsBoolean();
  }
}
