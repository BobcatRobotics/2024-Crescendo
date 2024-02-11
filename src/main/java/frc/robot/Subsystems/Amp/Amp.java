package frc.robot.Subsystems.Amp;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;
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
    

    public void setPos(){
        io.setPos(AmpConstants.rotationAmount);
    }
    public void stop(){
        io.stop();
    }
} 