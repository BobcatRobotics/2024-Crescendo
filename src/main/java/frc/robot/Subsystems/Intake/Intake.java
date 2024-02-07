package frc.robot.Subsystems.Intake;

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

    public void stop() {
        io.switchMotorStop();
        io.floorMotorStop();
        io.outsideMotorStop();
    }
}
