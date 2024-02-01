package frc.robot.Subsystems.Intake;

public class Intake {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void runIn() {
        io.outerMotorSetPercentOut(1);
        io.middleMotorSetPercentOut(1);
        io.innerMotorSetPercentOut(1);
    }

    public void runOut() {
        io.outerMotorSetPercentOut(-1);
        io.middleMotorSetPercentOut(-1);
        io.innerMotorSetPercentOut(-1);
    }

    public void stop() {
        io.outerMotorStop();
        io.middleMotorStop();
        io.innerMotorStop();
    }
}
