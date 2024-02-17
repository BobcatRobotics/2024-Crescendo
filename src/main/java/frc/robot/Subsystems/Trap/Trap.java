package frc.robot.Subsystems.Trap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapConstants;
import frc.robot.Subsystems.Climber.ClimberIOInputsAutoLogged;

public class Trap extends SubsystemBase {
    // Advantage Kit Logging
    private final ClimberIOInputsAutoLogged trapInterface = new ClimberIOInputsAutoLogged();

     TrapIO trap;

     public Trap(TrapIO trapModule){
        this.trap = trapModule;
     }

     public void periodic(){
        trap.updateInputs(trapInterface);
     }

}
