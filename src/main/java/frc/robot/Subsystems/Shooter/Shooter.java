package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateConfigs();
    }

    public void shoot() {
        io.bottomMotorSetPercentOut(1.0);
        io.topMotorSetPercentOut(1.0);
    }

    public void feedIn() {
        io.feederMotorSetPercentOut(1.0);
    }

    public void feedOut() {
        io.feederMotorSetPercentOut(-1.0);
    }

    public void increaseAngle() {
        io.angleMotorSetPercentOut(1.0);
    }

    public void decreaseAngle() {
        io.angleMotorSetPercentOut(-1.0);
    }

    public void stop() {
        io.topMotorStop();
        io.angleMotorStop();
        io.bottomMotorStop();
        io.feederMotorStop();
    }

}