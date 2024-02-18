package frc.robot.Subsystems.Amp;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Amp extends SubsystemBase {
    private final AmpIO io;
    private final AmpIOInputsAutoLogged inputs = new AmpIOInputsAutoLogged();

    public Amp(AmpIO io){
        this.io = io;
    }
    public void period(){
        io.updateInputs(inputs);
        Logger.processInputs("Amp", inputs);
    }
    

    public void setPos(double rot){
        io.setPos(rot); //rotationsss
    }

    public void setPercentOut(double percent){
        io.setPercent(percent);
    }

    public void stop(){
        io.stop();
    }
} 