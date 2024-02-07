package frc.robot.Subsystems.Intake;

import frc.robot.Constants.IntakeConstants;

public class Intake {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void intakeToShooter() {
        io.switchMotorSetPercentOut(1);
        io.floorMotorSetPercentOut(1);
        io.outsideMotorSetPercentOut(1);
    }

    public void intakeToTrap() {
        io.switchMotorSetPercentOut(-1);
        io.floorMotorSetPercentOut(1);
        io.outsideMotorSetPercentOut(1);
    }

    public void runOut() {
        io.switchMotorSetPercentOut(-1);
        io.floorMotorSetPercentOut(-1);
        io.outsideMotorSetPercentOut(-1);
    }

    public boolean hasPiece() {
        return inputs.tofValue < IntakeConstants.tofTresh;
    }

    public void stop() {
        io.switchMotorStop();
        io.floorMotorStop();
        io.outsideMotorStop();
    }
}
