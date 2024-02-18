package frc.robot.Subsystems.Trap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Climber.ClimberIOInputsAutoLogged;

public class Trap extends SubsystemBase {
    private final TrapIO io;
    private final TrapIOInputsAutoLogged inputs = new TrapIOInputsAutoLogged();

    public Trap(TrapIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Trap", inputs);
    }

    public void setRollerPercent(double percent) {
        io.setRollerPercent(percent);
    }

    public void stopRoller() {
        io.stopRollers();
    }

    public void setArmPercent(double percent) {
        io.setArmPercent(percent);
    }

    public void stopArm() {
        io.stopArm();
    }

}
