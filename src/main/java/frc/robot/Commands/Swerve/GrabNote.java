// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.Vision;


public class GrabNote extends Command {
  /** Creates a new grabNote. */

  private Vision vision;
  private Swerve swerve;

  private double kP=0.25;
  private double kPRotation = 0.025;
  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;
  private Pose2d notePos;

  public GrabNote(Swerve swerve, Vision vision) {
    this.swerve = swerve;
    addRequirements(swerve);
    this.vision = vision;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController = new PIDController(kP, 0,0);
    xController.setTolerance(0.4);
    yController = new PIDController(kP, 0,0);
    yController.setTolerance(0.4);
    thetaController = new PIDController(kPRotation, 0,0);
    thetaController.setTolerance(4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  if (vision.getTClass()==0){
    //if were more than 5 degrees off, only rotate, once were within 5 degrees, translate.
    notePos = vision.getNotePose();
     if(!(Math.abs(thetaController.getPositionError()) < 5)){
     swerve.drive(new Translation2d(), -thetaController.calculate(notePos.getRotation().getDegrees()),false,false);
     }else{
      swerve.drive(new Translation2d(0,xController.calculate(notePos.getX())), -thetaController.calculate(notePos.getRotation().getDegrees()),false,false);
     }
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
