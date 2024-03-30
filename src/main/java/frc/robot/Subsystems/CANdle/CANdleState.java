package frc.robot.Subsystems.CANdle;



/*

desired behavior, untested:

intaking - fire animation while held
intook - solid green 1 second
intake stall - unbound
reset pose - strobe gold 1 second
reset gyro - strobe gold 1 second
note hunting - rainbow while held
outtake - strobe orange while held
 */

public enum CANdleState {
    OFF,
    INTAKING,
    INTOOK,
    INTAKESTALL,
    RESETPOSE,
    NOTEHUNTING,
    RESETGYRO,
    OUTAKE
}
