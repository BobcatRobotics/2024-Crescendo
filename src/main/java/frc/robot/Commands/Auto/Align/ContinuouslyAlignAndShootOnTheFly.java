// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto.Align;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spivit.Spivit;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Util.BobcatUtil;

public class ContinuouslyAlignAndShootOnTheFly extends Command {
  private Swerve swerve;
  private Spivit spivit;
  private Shooter shooter;
  private Intake intake;
  private BooleanSupplier readyToFire;
  private DoubleSupplier shootTime;
  private BooleanSupplier spinUp;

  private Timer timer = new Timer();

  private double[] currentAngles;

  /** Creates a new ContinuouslyAlignAndShootOnTheFly. */
  public ContinuouslyAlignAndShootOnTheFly(Swerve swerve, Spivit spivit, Shooter shooter, Intake intake, BooleanSupplier readyToFire, DoubleSupplier shootTime, BooleanSupplier spinUp) {
    this.swerve = swerve;
    this.spivit = spivit;
    this.shooter = shooter;
    this.intake = intake;
    this.readyToFire = readyToFire;
    this.shootTime = shootTime;
    this.spinUp = spinUp;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spivit, shooter, intake);

    timer.stop();
    timer.reset();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAngles = swerve.getShootWhileMoveBallistics();
    swerve.setRotationTarget(Rotation2d.fromRadians(BobcatUtil.isRed() ? currentAngles[0] : currentAngles[0] + Math.PI));
    spivit.setAngle(currentAngles[1]);
    shooter.setSpeed(ShooterConstants.fastShooterRPMSetpoint, ShooterConstants.fastShooterRPMSetpoint);
    intake.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngles = swerve.getShootWhileMoveBallistics();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
