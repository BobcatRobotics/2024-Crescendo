package frc.robot.Subsystems.Trap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapConstants;
import frc.robot.Subsystems.Climber.ClimberIOInputsAutoLogged;

public class Trap extends SubsystemBase {
    // Advantage Kit Logging
    private final TrapIOInputsAutoLogged trapInterface = new TrapIOInputsAutoLogged();

     TrapIO trap;

     public Trap(TrapIO trapModule){
        this.trap = trapModule;
     }

    // Update the inputs periodically 
     public void periodic(){
        trap.updateInputs(trapInterface);
}

}
