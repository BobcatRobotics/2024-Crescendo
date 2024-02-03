package frc.robot.Subsystems.Amp;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Amp extends SubsystemBase {
    private final AmpIO io;

    public Amp(AmpIO io){
        this.io = io;
    }
    public void period(){}
    

    public void run(){
        io.run(Constants.AMPConstants.rotationAmount);
    }
    public void stop(){
        io.stop();
    }
} 