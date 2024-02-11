package frc.robot.Subsystems.Trap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapConstants;
import frc.robot.Subsystems.Climber.ClimberIOInputsAutoLogged;

public class Trap extends SubsystemBase {
    private final ClimberIOInputsAutoLogged climberInterface = new ClimberIOInputsAutoLogged();

     TrapIO trap;

     public Trap(){

     }

}
