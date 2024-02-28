// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.google.flatbuffers.FlexBuffers.Vector;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.limelightConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Vision.CamMode;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionIO;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public double visionStdDev;

  public edu.wpi.first.math.Vector<N3> stateStdDevs;

  @Override
  public void robotInit() {
    Pathfinding.setPathfinder(new LocalADStarAK()); //this must be first, for pathplanner and advantagekit interoperability
    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }
 
 
 
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

 

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    m_robotContainer = new RobotContainer();

    m_robotContainer.m_intakeVision.setCamMode(CamMode.VISION);
    m_robotContainer.m_intakeVision.setPipeline(LimelightConstants.intake.apriltagPipelineIndex);
    m_robotContainer.m_shooterLeftVision.setCamMode(CamMode.VISION);
    m_robotContainer.m_shooterLeftVision.setPipeline(LimelightConstants.shooterLeft.apriltagPipelineIndex);
    m_robotContainer.m_shooterRightVision.setCamMode(CamMode.VISION);
    m_robotContainer.m_shooterRightVision.setPipeline(LimelightConstants.shooterRight.apriltagPipelineIndex);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.configureBindings();  



  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.m_intakeVision.setCamMode(CamMode.DRIVERCAM);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
