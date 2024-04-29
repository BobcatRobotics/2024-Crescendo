package frc.lib.util.BobcatLib.Sysid;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Sysid {
    private SysidCompatibleSwerve swerve;
    private SysIdRoutine routine;
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
    public Sysid(SysidCompatibleSwerve swerve){
        this.swerve = swerve; //TODO: once we have a state machine, this should just be Swerve.getInstance()
        routine = new SysIdRoutine(new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism(
            (swerve.sysidVoltage()) -> {}, 
                   ));
    }


}
