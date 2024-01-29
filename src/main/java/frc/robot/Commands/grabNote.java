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


public class grabNote extends Command {
  /** Creates a new grabNote. */

  private String limelight;
  private Swerve swerve;
  private double xVal;
  private double yVal;
  private double kP;
  private double kI;
  private double kD;
  private PIDController pid;

  public grabNote(Swerve swerve, double xVal, String limelight) {
    this.swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
    this.xVal = xVal;
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      try (PIDController pid = new PIDController(kP, kI, kD)) {
        pid.setTolerance(0.4);
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      yVal = ((180*Constants.FieldConstants.noteDiameter)/(63.3*Math.PI)) * (1/((LimelightHelpers.getLimelightNTDouble(null, "thor"))/Constants.LimelightConstants.horPixles));
      swerve.drive(
              new Translation2d(1, pid.calculate(yVal, 0)).times(SwerveConstants.maxSpeed), 
              0,
              true);


      

      if(xVal<=swerve.getPoseEstimation().getX()){
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
