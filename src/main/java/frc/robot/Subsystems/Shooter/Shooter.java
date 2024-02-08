package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
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
        io.bottomMotorSetVelocityOut(0);
        io.topMotorSetVelocityOut(0);
    }


    public void increaseAngle() {
        io.angleMotorSetPosition(1.0);
    }

    public void decreaseAngle() {
        io.angleMotorSetPosition(-1.0);
    }

    

    public void alignAngle(Pose2d botpose, Pose2d speakerpose){
        double xdist = Math.abs(botpose.getTranslation().getDistance(speakerpose.getTranslation()));
        io.angleMotorSetPosition(0);
    }

    public void stop() {
        io.topMotorStop();
        io.angleMotorStop();
        io.bottomMotorStop();
    }


}