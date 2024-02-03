package frc.robot.Subsystems.Amp;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Amp.AmpIO.AmpIOInputs;

public class Amp extends SubsystemBase {
    private final AmpIO io;
    private final AmpIOInputs inputs = new AmpIOInputsAutoLogged();

    public Amp(AmpIO io){
        this.io = io;
    }
    public void period(){
        io.updateInputs(inputs);
    }
    

    public void run(){
        io.run(Constants.AMPConstants.rotationAmount);
    }
    public void stop(){
        io.stop();
    }
} 