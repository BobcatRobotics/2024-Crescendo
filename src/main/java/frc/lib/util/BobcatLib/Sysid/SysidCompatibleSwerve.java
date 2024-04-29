package frc.lib.util.BobcatLib.Sysid;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

public interface SysidCompatibleSwerve {
    public default Consumer<Voltage> sysidVoltage(){return Volts.of(0);}
}
