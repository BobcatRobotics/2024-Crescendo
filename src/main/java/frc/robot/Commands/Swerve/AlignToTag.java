// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Swerve;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.Vision;


public class AlignToTag extends Command {
  /** Creates a new grabNote. */

  private Vision vision;
  private Swerve swerve;

  private double kP=0.25;
  private double kPRotation = 0.025;
  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;
  private Translation2d dist;
  private Optional<Integer> tagID;
  private int currID;
  private AprilTag currTag;
  

  public AlignToTag(Swerve swerve, Vision vision, Optional<Integer> tagID) {
    this.swerve = swerve;
    this.vision = vision;
    this.tagID = tagID;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController = new PIDController(kP, 0,0);
    xController.setTolerance(0.4);
    yController = new PIDController(kP, 0,0);
    yController.setTolerance(0.4);
    yController.setSetpoint(1);
    thetaController = new PIDController(kPRotation, 0,0);
    thetaController.setTolerance(4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(vision.getTV()){
    currID = (int) LimelightHelpers.getFiducialID(null);
    }


    if(tagID != null && tagID.get() == currID){
      swerve.drive(new Translation2d(), -thetaController.calculate(vision.getTX().getDegrees()), false, false, false, 0);
    }else{
      // swerve.drive(thetaController.getPositionError() < 4? new Translation2d(0,yController.calculate(vision.getTranslationToTag(currID).getX())):new Translation2d(), -thetaController.calculate(vision.getTX().getDegrees()), false, false, false,0);
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
