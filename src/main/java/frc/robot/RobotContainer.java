// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Auto.AutoBreak;
import frc.robot.Commands.Auto.AutoIntake;
import frc.robot.Commands.Auto.ContinouslyAlignAndShoot;
import frc.robot.Commands.Auto.ReleaseHook;
import frc.robot.Commands.Auto.RevToRPM;
import frc.robot.Commands.Auto.SubwooferShot;
import frc.robot.Commands.Auto.Align.AlignAndRevPPOverride;
import frc.robot.Commands.Auto.Align.AlignAndShoot;
import frc.robot.Commands.Auto.Align.AlignAndShootPPOverride;
import frc.robot.Commands.Intake.TeleopIntake;
import frc.robot.Commands.Multi.SetAmp;
import frc.robot.Commands.Swerve.GrabNote;
import frc.robot.Commands.Swerve.TeleopSwerve;
import frc.robot.Commands.Climber.ClimbMode;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.LimelightConstants.intake;
import frc.robot.Subsystems.Amp.Amp;
import frc.robot.Subsystems.Amp.AmpIOFalcon;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Climber.ClimberIO;
import frc.robot.Subsystems.Climber.ClimberIOFalcon;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Intake.IntakeIOFalcon;
import frc.robot.Subsystems.Rumble.Rumble;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Shooter.ShooterIOFalcon;
import frc.robot.Subsystems.Spivit.Spivit;
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
import frc.robot.Subsystems.Vision.CamMode;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionIO;
import frc.robot.Subsystems.Vision.VisionIOLimelight;
import frc.robot.Util.BobcatUtil;

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
  // public final Trap m_trap;
  public final Climber m_climber;
  public final Rumble m_Rumble; // mmmmmmmm rumble
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
        m_intakeVision.setCamMode(CamMode.DRIVERCAM);
        m_shooterRightVision = new Vision(new VisionIOLimelight(LimelightConstants.shooterRight.constants));
        m_shooterLeftVision = new Vision(new VisionIOLimelight(LimelightConstants.shooterLeft.constants));
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
        // m_trap = new Trap(new TrapIOFalcon());
        m_climber = new Climber(new ClimberIOFalcon());
        m_Rumble = new Rumble();
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
        // m_trap = new Trap(new TrapIO() {
        // });
        m_climber = new Climber(new ClimberIOFalcon());
        // });
        m_Rumble = new Rumble();

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
        m_intakeVision.setCamMode(CamMode.DRIVERCAM);

        m_intake = new Intake(new IntakeIO() {
        });
        m_shooter = new Shooter(new ShooterIO() {
        });
        m_amp = new Amp(new AmpIOFalcon());
        m_Spivit = new Spivit(new SpivitIOFalcon());
        // m_trap = new Trap(new TrapIO() {
        // });
        m_climber = new Climber(new ClimberIOFalcon());
        // });
        m_Rumble = new Rumble();

        // m_Vision = new Vision(new VisionIOLimelight());
        break;

    }

    /*
     * Auto Events
     * 
     * Names must match what is in PathPlanner
     * Please give descriptive names
     */
    NamedCommands.registerCommand("StartShooting",
        new ContinouslyAlignAndShoot(m_swerve, m_Spivit, m_shooter, m_intake, () -> false, 5000));
    NamedCommands.registerCommand("SubwooferShot", new SubwooferShot(m_swerve, m_Spivit, m_shooter, 5000));
    NamedCommands.registerCommand("StopShooter", new InstantCommand(m_shooter::stop));
    NamedCommands.registerCommand("Intake", new AutoIntake(m_intake));
    NamedCommands.registerCommand("SpinUp", new InstantCommand(
        () -> m_shooter.setSpeed(ShooterConstants.fastShooterRPMSetpoint, ShooterConstants.fastShooterRPMSetpoint)));
    NamedCommands.registerCommand("ShootWhileBacking", new AlignAndShootPPOverride(m_swerve, m_Spivit, m_shooter, m_intake, 0.5, -2));
    NamedCommands.registerCommand("AlignAndShoot1.5", new AlignAndShoot(m_swerve, m_Spivit, m_shooter, m_intake, 1.5));
    NamedCommands.registerCommand("AlignAndShoot1.0", new AlignAndShoot(m_swerve, m_Spivit, m_shooter, m_intake, 1.0));
    NamedCommands.registerCommand("AlignAndShoot0.5", new AlignAndShoot(m_swerve, m_Spivit, m_shooter, m_intake, 0.5));
    // NamedCommands.registerCommand("LeftBiasedAlignAndShoot", new LeftBiasedAlignAndShoot(m_swerve, m_Spivit, m_shooter, m_intake) );
    NamedCommands.registerCommand("AlignDontShoot", new AlignAndRevPPOverride(m_swerve, m_Spivit, m_shooter)); //Aligns and revs, but doesnt feed, this command never ends, you will have to end it manually
    NamedCommands.registerCommand("Unhook", new ReleaseHook(m_Spivit));
    NamedCommands.registerCommand("StopIntake", new InstantCommand(m_intake::stop));
    NamedCommands.registerCommand("RevToRPM", new RevToRPM(m_shooter, ShooterConstants.fastShooterRPMSetpoint, 4500));
    // NamedCommands.registerCommand("Break", new AutoBreak(m_Spivit));
    /*
     * Auto Chooser
     * 
     * Names must match what is in PathPlanner
     * Please give descriptive names
     */
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("CenterShootNScoot", new PathPlannerAuto("centerShootNScoot"));
    autoChooser.addOption("KidsMeal", new PathPlannerAuto("AdjustedKidsMeal"));
    // autoChooser.addOption("OutOfTheWay", new PathPlannerAuto("out of the way"));
    autoChooser.addOption("OuttaTheWay2", new PathPlannerAuto("OuttaTheWay2"));
    autoChooser.addOption("OUT OF THE WAY 3", new PathPlannerAuto("Outta the way 3"));
    autoChooser.addOption("FastFood", new PathPlannerAuto("FastFood"));
    // autoChooser.addOption("Odometry Tuning", new PathPlannerAuto("Odometry Tuning"));
    // autoChooser.addOption("AdjustedKidsMeal", new
    // PathPlannerAuto("AdjustedKidsMeal"));

    configureBindings();
  }

  /**
   * IMPORTANT NOTE:
   * When a gamepad value is needed by a command, don't
   * pass the gamepad to the command, instead have the
   * constructor for the command take an argument that
   * is a supplier of the value that is needed. To supply
   * the values, use an anonymous function like this:
   * 
   * () -> buttonOrAxisValue
   */
  public void configureBindings() {
    /*
     * A for amp mode
     * B for stow (amp)
     * X is
     * Y is
     * D PAD UP is intake to trap
     * D PAD DOWN is intake to shooter
     * LEFT BUMPER is align
     * LEFT TRIGGER is climber down
     * RIGHT BUMPER is feed
     * RIGHT TRIGGER is climber up
     * SELECT is out take
     * LEFT STICK is trap scoring arm
     * RIGHT STICK is manual spivit
     * 
     * Buttons
     * 1 -b
     * 2 -a
     * 3- y
     * 4 -x
     * 5 -lb
     * 6 - rb
     * 7 - select
     * 8 - start
     * 9 - bl
     * 10 - br
     * 
     * Axes
     * 0 -
     * 1 -
     * 2 - LT
     * 3 - RT
     * 4 -
     * 5 -
     * 
     * Axis indices start at 0, button indices start at one -_-
     */

    /* Drive with joysticks */
    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve,
            () -> -strafe.getRawAxis(Joystick.AxisType.kY.value)
                * Math.abs(strafe.getRawAxis(Joystick.AxisType.kY.value)), // translation
            () -> -strafe.getRawAxis(Joystick.AxisType.kX.value)
                * Math.abs(strafe.getRawAxis(Joystick.AxisType.kX.value)), // strafe
            () -> -rotate.getRawAxis(Joystick.AxisType.kX.value), // rotate
            () -> false, // robot centric
            () -> -rotate.getRawAxis(Joystick.AxisType.kZ.value) * 0.2, // Fine strafe
            () -> -strafe.getRawAxis(Joystick.AxisType.kZ.value) * 0.2, // Fine translation
            () -> false, // align to amp
            gp.button(5) // align to speaker
        // () -> false
        ));
    // reset gyro
    rotate.button(1).onTrue(new InstantCommand(m_swerve::zeroGyro));

    /* Intake Controls */
    m_intake.setDefaultCommand(
        new TeleopIntake(
            m_intake,
            gp.povDown(), // shooter
            // gp.povUp(), //poptart
            // () -> (gp.button(5).getAsBoolean() && gp.povDown().getAsBoolean()), // if
            // holding spin up shooter button, run intake to fire
            gp.button(7), // outtake - 'back' button
            // () -> m_shooter.atSpeed(),
            // () -> m_shooter.atAngle()
            () -> true,
            () -> true,
            gp.button(6), // feed to shooter/manual override
            // m_trap,
            m_Rumble).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    new Trigger(() -> m_intake.hasPiece()).onTrue(
        Commands.sequence(new InstantCommand(() -> m_swerve.setLimeLEDS(true)),
            new WaitCommand(2),
            new InstantCommand(() -> m_swerve.setLimeLEDS(false))));

    gp.button(7).whileTrue(new StartEndCommand(() -> m_shooter.setSpeed(-1000, -1000), m_shooter::stop, m_shooter));

    /* Shooter Controls */
    // while button is held, rev shooter
    gp.button(10)
        .whileTrue(new RunCommand(
            () -> m_shooter.setSpeed(() -> BobcatUtil.getShooterSpeed(m_Spivit.getAngle(), m_amp.getAngle()),
                () -> BobcatUtil.getShooterSpeed(m_Spivit.getAngle(), m_amp.getAngle()))))
        .onFalse(new InstantCommand(m_shooter::stop)); // back right

    /* feed to opponents */
    gp.button(4).whileTrue(new RunCommand(() -> {
        m_Spivit.setAngle(ShooterConstants.ampPosition);
        m_shooter.setSpeed(3700, 3700);
    }, m_Spivit, m_shooter)).onFalse(new InstantCommand(m_shooter::stop).alongWith(new InstantCommand(m_Spivit::stopMotorFeedforward))); // x

    /* Spivit controls */
    // manual down
    gp.axisGreaterThan(5, .6)
        .whileTrue(new StartEndCommand(() -> m_Spivit.setPercent(-0.15), m_Spivit::stopMotorFeedforward, m_Spivit));
    // manual up
    gp.axisLessThan(5, -.6)
        .whileTrue(new StartEndCommand(() -> m_Spivit.setPercent(0.20), m_Spivit::stopMotorFeedforward, m_Spivit));
    // this sets it to a specific angle
    gp.button(5).whileTrue(new RunCommand(() -> m_Spivit.setAngle(m_swerve.calcAngleBasedOnHashMap()), m_Spivit))
        .onFalse(new InstantCommand(m_Spivit::stopMotorFeedforward));
    gp.button(9).whileTrue(new RunCommand(() -> m_Spivit.setAngle(ShooterConstants.subwooferShot), m_Spivit))
        .onFalse(new InstantCommand(m_Spivit::stopMotorFeedforward));

    /* amp controls */
    // retract
    gp.button(1).onTrue(new SetAmp(m_amp, m_Spivit, false).withInterruptBehavior(InterruptionBehavior.kCancelSelf)); // b
    // deploy
    gp.button(2).onTrue(new SetAmp(m_amp, m_Spivit, true).withInterruptBehavior(InterruptionBehavior.kCancelSelf)); // a
    // zero
    gp.button(3).onTrue(new InstantCommand(m_amp::zero)); // y

    // // shooter amp speed
    // gp.button(4).onTrue(new InstantCommand(() -> m_shooter.setSpeed(1800, 1800)))
    //     .onFalse(new InstantCommand(m_shooter::stop)); // x

    // manual
    // gp.axisGreaterThan(1, .6).whileTrue(new InstantCommand(() ->
    // m_amp.setPercentOut(0.05))).onFalse(new InstantCommand(() -> m_amp.stop()));
    gp.axisGreaterThan(1, .6).whileTrue(new StartEndCommand(() -> m_amp.setPercentOut(-0.1), m_amp::stopMotorStowPos, m_amp));
    // this runs it down
    // gp.axisLessThan(1, -.6).whileTrue(new InstantCommand(() ->
    // m_amp.setPercentOut(-0.05))).onFalse(new InstantCommand(() -> m_amp.stop()));
    gp.axisLessThan(1, -.6).whileTrue(new StartEndCommand(() -> m_amp.setPercentOut(0.1), m_amp::stop, m_amp));
    gp.button(8).onTrue(new InstantCommand(() -> m_swerve.resetPose(BobcatUtil.getAlliance() == Alliance.Blue
        ? new Pose2d(FieldConstants.blueSpeakerPose.plus(new Translation2d(1.3, 0)), Rotation2d.fromDegrees(0))
        : new Pose2d(FieldConstants.redSpeakerPose.plus(new Translation2d(-1.3, 0)), Rotation2d.fromDegrees(180)))));
    gp.axisGreaterThan(3, 0.07).whileTrue(new ClimbMode(m_climber, m_amp, m_Spivit, () -> -gp.getRawAxis(3)));

    /* trap controls */
    // gp.povRight().whileTrue(new StartEndCommand(() -> m_trap.setArmPercent(0.1),
    // m_trap::stopArm, m_trap));
    // gp.povLeft().whileTrue(new StartEndCommand(() -> m_trap.setArmPercent(-0.1),
    // m_trap::stopArm, m_trap));
    // gp.button(7).whileTrue(new StartEndCommand(() ->
    // m_trap.setRollerPercent(0.3), m_trap::stopRoller, m_trap));
    // gp.button(8).whileTrue(new StartEndCommand(() ->
    // m_trap.setRollerPercent(-0.3), m_trap::stopRoller, m_trap));
    // gp.button(9).whileTrue(new StartEndCommand(() ->
    // m_trap.setRollerPercent(0.3), m_trap::stopRoller, m_trap)); // back left
    // this drives it towards the robot, i think
    // gp.axisGreaterThan(1, .6).whileTrue(new StartEndCommand(() ->
    // m_trap.setArmPercent(0.1), m_trap::stopArm, m_trap));
    // this drives it towards the trap
    // gp.axisLessThan(1, -.6).whileTrue(new StartEndCommand(() ->
    // m_trap.setArmPercent(-0.1), m_trap::stopArm, m_trap));

    // climber controls
    // this raises the hooks
    // gp.axisGreaterThan(3, 0.07).whileTrue(new RunCommand(() ->
    // m_climber.setPercentOut(-gp.getRawAxis(3)), m_climber)).onFalse(new
    // InstantCommand(m_climber::stop));
    // this lowers the hooks
    gp.axisGreaterThan(2, 0.07).whileTrue(new RunCommand(() -> m_climber.setPercentOut(gp.getRawAxis(2)), m_climber))
        .onFalse(new InstantCommand(m_climber::stop));

    // gp.povLeft().whileTrue(new ParallelCommandGroup(
    //         new TeleopIntake(            
    //         m_intake,
    //         (gp.povLeft()), // shooter
    //         // gp.povUp(), //poptart
    //         // () -> (gp.button(5).getAsBoolean() && gp.povDown().getAsBoolean()), // if
    //         // holding spin up shooter button, run intake to fire
    //         null, // outtake - 'back' button
    //         // () -> m_shooter.atSpeed(),
    //         // () -> m_shooter.atAngle()
    //         () -> true,
    //         () -> true,
    //         (null), // feed to shooter/manual override
    //         // m_trap,
    //         m_Rumble).withInterruptBehavior(InterruptionBehavior.kCancelSelf), 
    //         new GrabNote(m_swerve, m_intakeVision)));
    gp.povLeft().whileTrue(new GrabNote(m_swerve, m_intakeVision, true, m_intake));


    /* Drive with gamepad */
    // m_swerve.setDefaultCommand(
    // new TeleopSwerve(
    // m_swerve,
    // () -> -gp.getRawAxis(Joystick.AxisType.kY.value)
    // * Math.abs(gp.getRawAxis(Joystick.AxisType.kY.value)),
    // () -> -gp.getRawAxis(Joystick.AxisType.kX.value)
    // * Math.abs(gp.getRawAxis(Joystick.AxisType.kX.value)),
    // () -> -gp.getRawAxis(Joystick.AxisType.kZ.value),
    // () -> false,
    // () -> 0.0,
    // () -> 0.0,
    // () -> false
    // ));
    // gp.button(1).onTrue(new InstantCommand(m_swerve::zeroGyro));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
