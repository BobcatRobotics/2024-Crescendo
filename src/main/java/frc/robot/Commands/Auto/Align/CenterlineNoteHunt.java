// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto.Align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.Vision;

public class CenterlineNoteHunt extends Command {
  private Vision vision;
  private Swerve swerve;

  private double kP=0.5;
  private double kPRotation = 0.025;
  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;
  private PIDController initialTurnController;
  private Pose2d notePos;
  private boolean intakeNote;
  private Intake intake;
  private double initialTurn; //radians, field oriented
  private boolean finished = false;

  /** Creates a new CenterlineNoteHunt. */
  public CenterlineNoteHunt(Swerve swerve, Vision vision, boolean intakeNote, Intake intake, double initialTurn) {
    this.swerve = swerve;
    addRequirements(swerve, intake);
    this.vision = vision;
    this.intakeNote=intakeNote;
    this.intake=intake;
    this.initialTurn=initialTurn;
    

    // Use addRequirements() here to declare subsystem dependencies.
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
    thetaController.enableContinuousInput(0, 360);
    initialTurnController= new PIDController(kPRotation, 0, 0);
    initialTurnController.setTolerance(Units.degreesToRadians(3));
    initialTurnController.enableContinuousInput(0, 2*Math.PI);
    swerve.drive(new Translation2d(), initialTurnController.calculate(swerve.getPose().getRotation().getRadians(),initialTurn), true, false, false, 0, false, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.hasPiece()){
      intake.stop();
      finished = true;
    }
    else{
      if (vision.getTClass()==0){
        notePos = vision.getNotePose();
        swerve.drive(new Translation2d(xController.calculate(-notePos.getX())*2.5,yController.calculate(notePos.getY())), thetaController.calculate(notePos.getRotation().getDegrees()),false,false,false,0,false,0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
