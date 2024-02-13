package frc.robot.Subsystems.Shooter;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/toprpm", inputs.topMotorVelocityRPS*60);
        Logger.recordOutput("Shooter/bottomrpm", inputs.bottomMotorVelocityRPS*60);
    }

    public void setSpeed(double rps) {
        io.setTopVelocity(rps);
        io.setBottomVelocity(rps);
    }
    public void setVelocityTune(double rpm){
        io.setVelocityTune(rpm);
    }

    public void setAngle(double degrees) {
        io.setAngle(degrees);
    }

    /**
     * 
     * @param botpose
     * @param speakerpose
     * @return returns in RADIANS?!?!??! what the flip?
     */
    public double getAngleToSpeaker(Pose2d botpose, Pose2d speakerpose){
        double xDist = Math.abs(botpose.getTranslation().getDistance(speakerpose.getTranslation()));
        double yDist = FieldConstants.speakerHeight;
        return Math.atan(yDist/xDist);
    }

    public void stop() {
        io.stopTopMotor();
        io.stopBottomMotor();
    }

    public void stopAngle(){
        io.stopAngleMotor();
    }

    public void setPercentOut(double percent){
        io.setPercentOut(percent);
    }



}