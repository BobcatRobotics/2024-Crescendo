// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spivit.Spivit;

public class AutoShoot extends Command {
  private Shooter shooter;
  private Spivit spivit;
  private Intake intake;

  private boolean ready = false;
  private Timer timer = new Timer();
  private double feedTime = 1;
  private boolean done = false;

  /** Creates a new AutoShoot. */
  public AutoShoot(Shooter shooter, Spivit spivit, Intake intake) {
    this.shooter = shooter;
    this.spivit = spivit;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, spivit, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.stop();
    shooter.setSpeed(ShooterConstants.fastShooterRPMSetpoint, ShooterConstants.fastShooterRPMSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ready && !timer.hasElapsed(feedTime)) {
      intake.intakeToShooter();
    } else if (timer.hasElapsed(feedTime)) {
      ready = false;
      intake.stop();
      done = true;
      timer.stop();
    } else if (shooter.atSpeed() && spivit.aligned()) {
      ready = true;
      timer.reset();
      timer.start();
      intake.intakeToShooter();
    } else {

    }
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
