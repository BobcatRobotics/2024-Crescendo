package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private double pieceTimestamp;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        SmartDashboard.putBoolean("has piece", hasPiece());
    }

    public void intakeToShooter() {
        io.switchMotorSetPercentOut(.75); //ID 9
        io.floorMotorSetPercentOut(1); //ID 10
        io.outsideMotorSetPercentOut(.75); //ID 11
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
    }

    public boolean hasPiece() {
        if (inputs.tofValue <= IntakeConstants.tofTresh) {
            pieceTimestamp = Timer.getFPGATimestamp();
            return true;
        } else if (pieceTimestamp + 1 > Timer.getFPGATimestamp()) {
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
