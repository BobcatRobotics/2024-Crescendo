// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.Vision;


public class grabNote extends Command {
  /** Creates a new grabNote. */

  private Vision vision;
  private Swerve swerve;
  private double yVal;
  private double kPx=0.1;
  private double kIx=0;
  private double kDx;
  private double kPr=0.1;
  private double kIr=0;
  private double kDr=0;
  private double kPy=0.1;
  private double kIy=0;
  private double kDy=0;
  private PIDController xpid;
  private PIDController rpid;
  private PIDController ypid;

  public grabNote(Swerve swerve, double yVal, Vision vision) {
    this.swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
    this.yVal = yVal;
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      try (PIDController xpid = new PIDController(kPx, kIx, kDx)) {
        xpid.setTolerance(0.4);
      }
      try (PIDController rpid = new PIDController(kPr, kIr, kDr)) {
        rpid.setTolerance(0.4);
      }
      try (PIDController ypid = new PIDController(kPy, kIy, kDy)) {
        ypid.setTolerance(0.4);
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (vision.getTClass()==0){
    swerve.drive(
            vision.getNoteTranslation(xpid,ypid), 
            rpid.calculate(vision.getRotation(),0),
            false,
            true);
    }


    if(yVal<=swerve.getPoseEstimation().getY()){
      end(true);
    }
    
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
