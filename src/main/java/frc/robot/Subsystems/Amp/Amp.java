package frc.robot.Subsystems.Amp;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;


public class Amp extends SubsystemBase {
    private final AmpIO io;
    private final AmpIOInputsAutoLogged inputs = new AmpIOInputsAutoLogged();

    public Amp(AmpIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Amp", inputs);
    }
    

    public void setPos(double degrees){
        io.setPos(degrees/360.0); // degrees -> rotationsss
    }

    public void setPosWithFeedforward(double degrees, double ff) {
        io.setPosWithFeedforward(degrees/360.0, ff);
    }

    public void deploy(){
        setPos(AmpConstants.deployValue);
    }
    public void retract(){
        setPosWithFeedforward(AmpConstants.retractValue, -0.06);
    }

    /**
     * 
     * @return whether or not the amp has reached its deploy setpoint
     */
    public boolean deployed(){
        return ( inputs.motorPosition >= AmpConstants.deployValue-3);
    }

    public boolean retracted(){
        return inputs.motorPosition < AmpConstants.retractValue+AmpConstants.retractTolerance;
    }

    public void setPercentOut(double percent){
        io.setPercent(percent);
    }

    public void stop(){
        io.stop();
    }

    public void zero() {
        io.zeroPosition();
    }

    public double getAngle(){
        return inputs.motorPosition;
    }

    
    public void stopMotorFeedforward(){
        io.stopMotorFeedforward();
    }

    public void stopMotorStowPos() {
        io.stopMotorStowPos();
    }
} 