package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private boolean intook = false;
    private boolean lastIntakeSensorValue = false;
    private boolean currentIntakeSensorValue = false;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void removePeice(){
        intook = false;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        SmartDashboard.putBoolean("has piece", hasPiece() || intook || inputs.intakeSensorTripped);
        Logger.recordOutput("Intake/HasPiece", hasPiece());
        lastIntakeSensorValue = currentIntakeSensorValue;
        currentIntakeSensorValue = inputs.intakeSensorTripped;
    }

    public void intakeToShooter() {
        io.switchMotorSetPercentOut(.75); //ID 9
        io.floorMotorSetPercentOut(1); //ID 10
        io.outsideMotorSetPercentOut(.85); //ID 11 // 0.75
        intook = false;
    }

    // .4 good speed for trap
    public void intakeToTrap() {
        io.switchMotorSetPercentOut(-.4);
        io.floorMotorSetPercentOut(.4);
        io.outsideMotorSetPercentOut(.4);
    }

    public void runOut() {
        io.switchMotorSetPercentOut(-.4);
        io.floorMotorSetPercentOut(-.4);
        io.outsideMotorSetPercentOut(-.4);
        intook = false;
    }

    @AutoLogOutput
    public boolean hasPiece() {
       /*  if (inputs.tofValue <= IntakeConstants.tofTresh || intook) {
            // intook = true;
            // return true;
            return false;
        } else */
        if ((!currentIntakeSensorValue && lastIntakeSensorValue) || intook) {
            intook = true;
            return true;
        }
        return false;
    }

    public void stop() {
        io.switchMotorStop();
        io.floorMotorStop();
        io.outsideMotorStop();
    }
}
