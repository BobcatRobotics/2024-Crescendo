// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Amp.Amp;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Rumble.Rumble;
import frc.robot.Subsystems.Spivit.Spivit;


public class ClimbMode extends Command {
    private Climber climber;
    private Amp amp;
    //private Rumble rumble;
    private Spivit spivit;
    private final DoubleSupplier rawAxis;

  /** Creates a new AutoClimbMode. */
  public ClimbMode(Climber climber, Amp amp, Spivit spivit, DoubleSupplier rawAxis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.amp = amp;
    //this.rumble=rumble;
    this.spivit = spivit;
    this.rawAxis = rawAxis;
    addRequirements(climber, amp, spivit);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.setPos(AmpConstants.deployValue);
    spivit.setAngle(ShooterConstants.releasehookSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setPercentOut(rawAxis.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spivit.stopMotorFeedforward();
    climber.stop();
    amp.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(climber.getPos() <= ClimberConstants.topLimit && 
    amp.getAngle()<=AmpConstants.deployValue && 
    spivit.getAngle() <= ShooterConstants.releasehookSetpoint);
  }
}
