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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Swerve.GyroIO;
import frc.robot.Subsystems.Swerve.GyroIOPigeon2;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveModuleIO;
import frc.robot.Subsystems.Swerve.SwerveModuleIOFalcon;
import frc.robot.Subsystems.Swerve.SwerveModuleIOSim;

public class RobotContainer {
  /* Joysticks + Gamepad */
  // private final CommandJoystick rotate = new CommandJoystick(0);
  // private final CommandJoystick strafe = new CommandJoystick(1);
  public final CommandJoystick gp = new CommandJoystick(2);

  /* Subsystems */
  public final Swerve m_swerve;

  /* Commands */

  /* Shuffleboard Inputs */
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        m_swerve = new Swerve(new GyroIOPigeon2(),
            new SwerveModuleIOFalcon(SwerveConstants.Module0Constants.constants),
            new SwerveModuleIOFalcon(SwerveConstants.Module1Constants.constants),
            new SwerveModuleIOFalcon(SwerveConstants.Module2Constants.constants),
            new SwerveModuleIOFalcon(SwerveConstants.Module3Constants.constants));
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        m_swerve = new Swerve(new GyroIO() {
        },
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim(),
            new SwerveModuleIOSim());
        break;

      // Replayed robot, disable IO implementations
      default:
        m_swerve = new Swerve(new GyroIO() {
        },
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {});
        break;
    }

    /* Auto Chooser */
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Path 1", new PathPlannerAuto("Auto 1"));

    /* Auto Events */
    NamedCommands.registerCommand("Auto Event", new InstantCommand());

    configureBindings();
  }

  private void configureBindings() {
    // m_swerve.setDefaultCommand(
    //     new TeleopSwerve(
    //         m_swerve,
    //         -strafe.getRawAxis(Joystick.AxisType.kY.value)
    //             * Math.abs(strafe.getRawAxis(Joystick.AxisType.kY.value)),
    //         -strafe.getRawAxis(Joystick.AxisType.kX.value)
    //             * Math.abs(strafe.getRawAxis(Joystick.AxisType.kX.value)),
    //         -rotate.getRawAxis(Joystick.AxisType.kX.value),
    //         false,
    //         -rotate.getRawAxis(Joystick.AxisType.kZ.value) * 0.2, // Fine tune
    //         -strafe.getRawAxis(Joystick.AxisType.kZ.value) * 0.2  // Fine tune
    //     ));
    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve,
            -gp.getRawAxis(1)
                * Math.abs(gp.getRawAxis(1)),
            -gp.getRawAxis(0)
                * Math.abs(gp.getRawAxis(0)),
            -gp.getRawAxis(2),
            false,
            0.0,
            0.0
        ));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
