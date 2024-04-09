// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import java.util.function.DoubleSupplier;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Spivit.Spivit;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Util.BobcatUtil;

public class AutosGrabNote extends Command {
  /** Creates a new grabNote. */

  private Vision vision;
  private Swerve swerve;
  private Spivit spivit;

  private double kP = 0.5;
  private double kPRotation = 0.05;
  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;
  private Pose2d notePos;
  private Intake intake;
  private boolean finished;

  public AutosGrabNote(Swerve swerve, Vision vision, Intake intake, Spivit spivit) {
    this.swerve = swerve;
    addRequirements(swerve, intake);
    this.vision = vision;
    this.intake = intake;
    this.spivit = spivit;
    finished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController = new PIDController(kP, 0, 0);
    xController.setTolerance(0.4);
    yController = new PIDController(kP, 0, 0);
    yController.setTolerance(0.4);
    thetaController = new PIDController(kPRotation, 0, 0);
    thetaController.setTolerance(4);
    thetaController.enableContinuousInput(0, 360);
    finished = false;
    spivit.setAngle(ShooterConstants.stow);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (BobcatUtil.isBlue() ? swerve.getPose().getX() > FieldConstants.centerlineX + 0.75
        : swerve.getPose().getX() < FieldConstants.centerlineX - 0.75) {
      finished = true;
    }

    if (!intake.hasPiece()) {
      intake.intakeToShooter();
    } else {
      intake.stop();
      finished = true;
    }

    if (vision.getTClass() == 0) {

      notePos = vision.getNotePose();
      if(thetaController.getPositionError() > 0.5){
      swerve.drive(new Translation2d(0,0), thetaController.calculate(notePos.getRotation().getDegrees()),false,false,false,0);
      }else{
      swerve.drive(new Translation2d(xController.calculate(-notePos.getX())*2.5,yController.calculate(notePos.getY())), thetaController.calculate(notePos.getRotation().getDegrees()),false,false,false,0);
      //swerve.drive(new Translation2d(xController.calculate(-notePos.getX())*2.5,0), thetaController.calculate(notePos.getRotation().getDegrees()),false,false,false,0);
      }
      swerve.setLastMovingYaw(swerve.getYaw().getRadians());
    } else {
      //TODO scan for notes
      finished = true;
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
