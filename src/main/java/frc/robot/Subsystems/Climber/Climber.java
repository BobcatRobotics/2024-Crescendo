package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private ClimberIO io;

    public Climber(ClimberIO io){
        this.io = io;
    }

    // Update the inputs periodically 
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("climber", inputs);
    }

    public void setPercentOut(double percent) {
        io.setPercentOut(percent);
    }

    public void stop() {
        io.stop();
    }
}
