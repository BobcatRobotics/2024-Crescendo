package frc.lib.util.BobcatLib.PID;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class DynamicTunePID extends PIDController {
    private double p;
    private double i;
    private double d;

    /**
     * WIP
     * 
     * Should automatically publish pid to nt for tuning purposes, NOT FOR USE IN COMP
     * robot will prob need to be in test mode for this to work
     */
    public DynamicTunePID(double kp, double ki, double kd){
        super(kp, ki, kd);
        p = kp;
        i = ki;
        d = kd;
        Shuffleboard.getTab("tuning").add(this).withWidget(BuiltInWidgets.kPIDController);
    }

    //public void setPID(){
    //    setPID(p, i, d);
    //}

    
    
}
