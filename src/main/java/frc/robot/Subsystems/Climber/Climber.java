package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private ClimberIO io;

    private double percent = 0;
    private double holdingPos = 0;

    public Climber(ClimberIO io){
        this.io = io;
        holdingPos = inputs.climberMotorPosition;
    }

    // Update the inputs periodically 
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("climber", inputs);
        if (percent == 0) {
            io.holdPos(holdingPos);
        } else {
            holdingPos = inputs.climberMotorPosition;
        }
    }

    public void setPercentOut(double percent) {
        io.setPercentOut(percent);
        this.percent = percent;
    }

    public void stop() {
        io.stop();
        percent = 0;
    }
}
