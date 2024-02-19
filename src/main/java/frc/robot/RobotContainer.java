// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.Intake.TeleopIntake;
import frc.robot.Commands.Swerve.TeleopSwerve;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Amp.Amp;
import frc.robot.Subsystems.Amp.AmpIO;
import frc.robot.Subsystems.Amp.AmpIOFalcon;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Climber.ClimberIO;
import frc.robot.Subsystems.Climber.ClimberIOFalcon;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOFalcon;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterIOFalcon;
import frc.robot.Subsystems.Spivit.Spivit;
import frc.robot.Subsystems.Spivit.SpivitIO;
import frc.robot.Subsystems.Spivit.SpivitIOFalcon;
import frc.robot.Subsystems.Swerve.GyroIO;
import frc.robot.Subsystems.Swerve.GyroIOPigeon2;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveModuleIO;
import frc.robot.Subsystems.Swerve.SwerveModuleIOFalcon;
import frc.robot.Subsystems.Swerve.SwerveModuleIOSim;
import frc.robot.Subsystems.Trap.Trap;
import frc.robot.Subsystems.Trap.TrapIO;
import frc.robot.Subsystems.Trap.TrapIOFalcon;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionIO;
import frc.robot.Subsystems.Vision.VisionIOLimelight;

public class RobotContainer {
  /* Joysticks + Gamepad */
  private final CommandJoystick rotate = new CommandJoystick(1);
  private final CommandJoystick strafe = new CommandJoystick(0);
  private final CommandJoystick gp = new CommandJoystick(2);

  /* Subsystems */
  public final Swerve m_swerve;
  public final Vision m_shooterLeftVision;
  public final Vision m_intakeVision;
  public final Vision m_shooterRightVision;
  public final Intake m_intake;
  public final Shooter m_shooter;
  public final Amp m_amp;
  public final Spivit m_Spivit;
  public final Trap m_trap;
  public final Climber m_climber;
  // public final Vision m_Vision;

  /* Commands */

  /* Shuffleboard Inputs */
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        m_intakeVision = new Vision(new VisionIOLimelight(LimelightConstants.intake.constants));// need index and
                                                                                                // limelight constants
                                                                                                // for the IO
        m_shooterRightVision = new Vision(new VisionIOLimelight(LimelightConstants.shooterRight.constants));
        m_shooterRightVision.setPipeline(LimelightConstants.shooterLeft.name, LimelightConstants.shooterRight.apriltagPipelineIndex);
        m_shooterLeftVision = new Vision(new VisionIOLimelight(LimelightConstants.shooterLeft.constants));
        m_shooterRightVision.setPipeline(LimelightConstants.shooterRight.name, LimelightConstants.shooterRight.apriltagPipelineIndex);
        m_swerve = new Swerve(new GyroIOPigeon2(),
            new SwerveModuleIOFalcon(SwerveConstants.Module0Constants.constants),
            new SwerveModuleIOFalcon(SwerveConstants.Module1Constants.constants),
            new SwerveModuleIOFalcon(SwerveConstants.Module2Constants.constants),
            new SwerveModuleIOFalcon(SwerveConstants.Module3Constants.constants),
            m_intakeVision, m_shooterLeftVision, m_shooterRightVision);
        m_intake = new Intake(new IntakeIOFalcon());
        m_shooter = new Shooter(new ShooterIOFalcon());
        m_amp = new Amp(new AmpIOFalcon());
        m_Spivit = new Spivit(new SpivitIOFalcon());
        m_trap = new Trap(new TrapIOFalcon());
        m_climber = new Climber(new ClimberIOFalcon());
        // m_Vision = new Vision(new VisionIOLimelight());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        m_intakeVision = new Vision(new VisionIO() {
        });
        m_shooterLeftVision = new Vision(new VisionIO() {
        });
        m_shooterRightVision = new Vision(new VisionIO() {
        });
        m_swerve = new Swerve(new GyroIO() {
        },
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            m_intakeVision, m_shooterLeftVision, m_shooterRightVision);

        m_intake = new Intake(new IntakeIO() {
        });
        m_shooter = new Shooter(new ShooterIO() {
        });
        // m_Vision = new Vision(new VisionIOLimelight());
        m_amp = new Amp(new AmpIOFalcon());
        m_Spivit = new Spivit(new SpivitIOFalcon());
        m_trap = new Trap(new TrapIO() {
        });
        m_climber = new Climber(new ClimberIO() { 
        });

        break;

      // Replayed robot, disable IO implementations
      default:
        m_intakeVision = new Vision(new VisionIO() {
        });
        m_shooterLeftVision = new Vision(new VisionIO() {
        });
        m_shooterRightVision = new Vision(new VisionIO() {
        });
        m_swerve = new Swerve(new GyroIO() {
        },
            new SwerveModuleIO() {
            },
            new SwerveModuleIO() {
            },
            new SwerveModuleIO() {
            },
            new SwerveModuleIO() {
            },
            m_intakeVision, m_shooterLeftVision, m_shooterRightVision);
        m_intake = new Intake(new IntakeIO() {
        });
        m_shooter = new Shooter(new ShooterIO() {
        });
        m_amp = new Amp(new AmpIOFalcon());
        m_Spivit = new Spivit(new SpivitIOFalcon());
        m_trap = new Trap(new TrapIO() {
        });
        m_climber = new Climber(new ClimberIO() { 
        });

        // m_Vision = new Vision(new VisionIOLimelight());
        break;

    }

    /*
     * Auto Chooser
     * 
     * Names must match what is in PathPlanner
     * Please give descriptive names
     */
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("outta the way", new PathPlannerAuto("Auto 1"));
    autoChooser.addOption("straight line", new PathPlannerAuto("straight line"));

    /*
     * Auto Events
     * 
     * Names must match what is in PathPlanner
     * Please give descriptive names
     */
    NamedCommands.registerCommand("Auto Event", new InstantCommand());

    configureBindings();
    SmartDashboard.putNumber("ShooterRPM", 0);
  }

  /*
   * IMPORTANT NOTE:
   * When a gamepad value is needed by a command, don't
   * pass the gamepad to the command, instead have the
   * constructor for the command take an argument that
   * is a supplier of the value that is needed. To supply
   * the values, use an anonymous function like this:
   * 
   * () -> buttonOrAxisValue
   */
  private void configureBindings() {
    /* Drive with joysticks */
    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve,
            () -> -strafe.getRawAxis(Joystick.AxisType.kY.value)
                * Math.abs(strafe.getRawAxis(Joystick.AxisType.kY.value)),
            () -> -strafe.getRawAxis(Joystick.AxisType.kX.value)
                * Math.abs(strafe.getRawAxis(Joystick.AxisType.kX.value)),
            () -> -rotate.getRawAxis(Joystick.AxisType.kX.value),
            () -> false,
            () -> -rotate.getRawAxis(Joystick.AxisType.kZ.value) * 0.2, // Fine tune
            () -> -strafe.getRawAxis(Joystick.AxisType.kZ.value) * 0.2, // Fine tune
            // strafe.button(1) 
            () -> false
        ));
    rotate.button(1).onTrue(new InstantCommand(m_swerve::zeroGyro));

    //strafe.button(1).onTrue(new DriveToPose(m_swerve));

      
    /* Intake Controls */
    m_intake.setDefaultCommand(
        new TeleopIntake(
            m_intake, 
            gp.povDown(), 
            gp.povUp(),
            () -> (gp.button(5).getAsBoolean() && gp.povDown().getAsBoolean()), // if holding spin up shooter button, run intake to fire
            gp.button(9), // back button
            // () -> m_shooter.atSpeed(),
            // () -> m_shooter.atAngle()
            () -> true,
            () -> true
        ));
    // gp.povDown().whileTrue(new InstantCommand(m_intake::intakeToShooter)).onFalse(new InstantCommand(m_intake::stop));
    // gp.povUp().whileTrue(new InstantCommand(m_intake::intakeToTrap)).onFalse(new InstantCommand(m_intake::stop));
    // gp.button(9).whileTrue(new InstantCommand(m_intake::runOut)).onFalse(new InstantCommand(m_intake::stop)); // start

    /* Shooter Controls */
    gp.button(5).whileTrue(new InstantCommand(() -> m_shooter.setSpeed(5000, 5000))).onFalse(new InstantCommand(m_shooter::stop)); // left bumper
    
    //this moves it down
    gp.axisGreaterThan(3, .6).whileTrue(new StartEndCommand(() -> m_Spivit.setPercent(-0.05), m_Spivit::stopMotorFeedforward, m_Spivit));

    //this moves it up
    gp.axisLessThan(3, -.6).whileTrue(new StartEndCommand(() -> m_Spivit.setPercent(0.05), m_Spivit::stopMotorFeedforward, m_Spivit));

    //this sets it to a specific angle
    gp.button(2).whileTrue(new StartEndCommand(() -> m_Spivit.setAngle(ShooterConstants.subwooferShot), m_Spivit::stopMotorFeedforward, m_Spivit));


    // amp controls 

    //this runs it up
    //gp.axisGreaterThan(1, .6).whileTrue(new InstantCommand(() -> m_amp.setPercentOut(0.05))).onFalse(new InstantCommand(() -> m_amp.stop()));
    gp.axisGreaterThan(1, .6).whileTrue(new StartEndCommand(() -> m_amp.setPercentOut(-0.1), m_amp::stop, m_amp));

    //this runs it down
    //gp.axisLessThan(1, -.6).whileTrue(new InstantCommand(() -> m_amp.setPercentOut(-0.05))).onFalse(new InstantCommand(() -> m_amp.stop()));
     gp.axisLessThan(1, -.6).whileTrue(new StartEndCommand(() -> m_amp.setPercentOut(0.1), m_amp::stop, m_amp));



    // trap controls
    //gp.button(7).whileTrue(new StartEndCommand(() -> m_trap.setRollerPercent(0.3), m_trap::stopRoller, m_trap)); // left trigger
    //gp.axisGreaterThan(1, .6).whileTrue(new StartEndCommand(() -> m_trap.setArmPercent(-0.1), m_trap::stopArm, m_trap));
    //gp.axisLessThan(1, -.6).whileTrue(new StartEndCommand(() -> m_trap.setArmPercent(0.1), m_trap::stopArm, m_trap));

    // climber controls
    gp.button(1).whileTrue(new StartEndCommand(() -> m_climber.setPercentOut(0.75), m_climber::stop, m_climber));
    gp.button(4).whileTrue(new StartEndCommand(() -> m_climber.setPercentOut(-0.75), m_climber::stop, m_climber));
    
    // gp.button(5).whileTrue(new InstantCommand(() -> m_shooter.setAngle(ShooterConstants.safePosition + 5))).whileFalse(new InstantCommand(() -> m_shooter.setAngle(ShooterConstants.safePosition)));
    /* Drive with gamepad */
    // m_swerve.setDefaultCommand(
    //     new TeleopSwerve(
    //         m_swerve,
    //         () -> -gp.getRawAxis(Joystick.AxisType.kY.value)
    //             * Math.abs(gp.getRawAxis(Joystick.AxisType.kY.value)),
    //         () -> -gp.getRawAxis(Joystick.AxisType.kX.value)
    //             * Math.abs(gp.getRawAxis(Joystick.AxisType.kX.value)),
    //         () -> -gp.getRawAxis(Joystick.AxisType.kZ.value),
    //         () -> false,
    //         () -> 0.0,
    //         () -> 0.0,
    //         () -> false
    //     ));

    // gp.button(1).onTrue(new InstantCommand(m_swerve::zeroGyro));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
