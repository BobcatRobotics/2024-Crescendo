package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter {
    public final ShooterIO io;
    

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void periodic() {
        //io.updateInputs(inputs);
        //io.updateConfigs();
        //Logger.processInputs("Shooter", inputs);
    }
}
