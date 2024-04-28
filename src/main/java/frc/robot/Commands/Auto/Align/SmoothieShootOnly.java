// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto.Align;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;

public class SmoothieShootOnly extends Command {
  private Intake intake;
  private BooleanSupplier aligned;
  private double shootTime;
  private boolean finished = false;
  private Timer shootTimer = new Timer();
  /** Creates a new SmoothieShootOnly. */
  public SmoothieShootOnly(Intake intake, BooleanSupplier aligned, double shootTime) {
    this.intake=intake;
    this.aligned=aligned;
    this.shootTime=shootTime;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  Logger.recordOutput("AutoSmoothie/Aligned", aligned.getAsBoolean());
    if(aligned.getAsBoolean()){
      intake.intakeToShooter();
      shootTimer.start();
    }
    
    if(shootTimer.hasElapsed(shootTime)){
      finished=true;
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
