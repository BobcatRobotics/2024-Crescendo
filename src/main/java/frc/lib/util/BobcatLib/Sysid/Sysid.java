package frc.lib.util.BobcatLib.Sysid;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;


public class Sysid extends SubsystemBase {
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

        public Sysid(SysidCompatibleSwerve swerve) {
                this.swerve = swerve; // TODO: once we have a state machine, this should just be Swerve.getInstance()
                routine = new SysIdRoutine(new SysIdRoutine.Config(),
                                new SysIdRoutine.Mechanism(
                                                (swerve::sysidVoltage),
                                                this::logMotors,
                                                this));
        }

    private void logMotors(SysIdRoutineLog log){
        log.motor("drive-left")
                                .voltage(m_appliedVoltage.mut_replace(swerve.getModuleVoltage(3) * RobotController.getBatteryVoltage(), Volts))
                                .linearPosition(m_distance.mut_replace(swerve.getModuleDistance(3), Meters))
                                .linearVelocity(m_velocity.mut_replace(swerve.getModuleSpeed(3), MetersPerSecond));
        log.motor("drive-right")
                                .voltage(m_appliedVoltage.mut_replace(swerve.getModuleVoltage(0) * RobotController.getBatteryVoltage(),Volts))
                                .linearPosition(m_distance.mut_replace(swerve.getModuleDistance(0), Meters))
                                .linearVelocity(m_velocity.mut_replace(swerve.getModuleSpeed(0),MetersPerSecond));
    }

    /**
     * REMEMBER TO ENSURE ALL MODULES ARE POINTING FORWARD
     */
    public Command getSysidTest(SysidTest test){
        switch (test) {
            case QUASISTATIC_FORWARD:
                return routine.quasistatic(Direction.kForward);
            case QUASISTATIC_BACKWARD:
                return routine.quasistatic(Direction.kReverse);
            case DYNAMIC_FORWARD:
                return routine.dynamic(Direction.kForward);
            case DYNAMIC_BACKWARD:
                return routine.dynamic(Direction.kReverse);
            default:
                return routine.quasistatic(Direction.kForward);
        }
    }

}
