package frc.lib.util.BobcatLib.Sysid;


import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

public interface SysidCompatibleSwerve {

    /**
     * set all modules to supplied voltage
     */
    public default void sysidVoltage(Measure<Voltage> volts){}
    
    /**
     * volts
     */
    public default double getModuleVoltage(int moduleNumber){
        return 0;
    }
    /**
     * meters
     */
    public default double getModuleDistance(int moduleNumber){
        return 0;
    }
    /**
     * meters/sec
     */
    public default double getModuleSpeed(int moduleNumber){
        return 0;
    }
}
