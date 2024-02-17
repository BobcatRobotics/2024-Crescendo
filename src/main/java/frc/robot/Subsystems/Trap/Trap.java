package frc.robot.Subsystems.Trap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapConstants;
import frc.robot.Subsystems.Climber.ClimberIOInputsAutoLogged;

/*
 * So far, with these functions, you can:
 * 1. Run shooter motors at a given velocity
 * 2. Make the trap motor go up
 * 3. Make the trap motor come down
 * 4. Stop the shooter on the trap
 */

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

    public void startShooter(double speed) // give it speed in rps 
    {
        trap.runShooterMotor(speed);
    }

    public void stopShooterMotor(){
        trap.stopShooterMotor();
    }

    public void extendTrap(double rotationAmount){
        trap.extendTrap(rotationAmount);
        trap.stopTrapMotion();
    }

    public void retractTrap(double rotationAmount){
        trap.inverseTrapDirection();
        trap.retractTrap(rotationAmount);
        trap.stopTrapMotion();
    }

}
