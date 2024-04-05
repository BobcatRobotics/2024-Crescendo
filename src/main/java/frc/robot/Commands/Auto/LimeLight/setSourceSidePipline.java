// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto.LimeLight;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Util.BobcatUtil;

public class setSourceSidePipline extends Command {
  /** Creates a new setSourceSidePipline. */
  private Vision limelightleft;
  private Vision limelightright;
  private Vision limelightcenter;
  private boolean finished = false;
  public setSourceSidePipline(Vision limelightleft, Vision limelightright, Vision limelightcenter) {
    this.limelightleft=limelightleft;
    this.limelightright=limelightright;
    this.limelightcenter=limelightcenter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(BobcatUtil.getAlliance()==Alliance.Blue){
    limelightright.setPipeline(Constants.LimelightConstants.shooterCenter.fpsPipline);
    }else{
    limelightleft.setPipeline(Constants.LimelightConstants.shooterCenter.fpsPipline);
    }

    limelightcenter.setPipeline(Constants.LimelightConstants.shooterCenter.resPipline);

    finished = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
