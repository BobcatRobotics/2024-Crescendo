package frc.robot.Subsystems.Shooter;


import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.BobcatUtil;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private double rpsTopSetpoint;
    private double rpsBotSetpoint;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/toprpm", inputs.topMotorVelocityRPS*60);
        Logger.recordOutput("Shooter/bottomrpm", inputs.bottomMotorVelocityRPS*60);
    }


    /**
     * 
     * @param rpmTop IN ROTATIONS PER MINUTE!!!!!!!!!!!!
     * @param rpmBot PER MINUTE YOU FOOL, NOT PER SECOND, PER MINUTEEEEEEEE!!!!!!!!
     */
    public void setSpeed(double rpmTop, double rpmBot) {
        this.rpsTopSetpoint = rpmTop/60;
        this.rpsBotSetpoint = rpmBot/60;
        io.setTopVelocity(rpmTop/60);
        io.setBottomVelocity(rpmBot/60);
    }

    public void setSpeed(DoubleSupplier rpmTop, DoubleSupplier rpmBot) {
        this.rpsTopSetpoint = rpmTop.getAsDouble()/60;
        this.rpsBotSetpoint = rpmBot.getAsDouble()/60;
        io.setTopVelocity(rpmTop.getAsDouble()/60);
        io.setBottomVelocity(rpmBot.getAsDouble()/60);
    }

    public void setSpeedBasedOnAngle(double spivitAngle, double ampAngle){
        double shooterSpeed = BobcatUtil.getShooterSpeed(spivitAngle, ampAngle); 
        setSpeed(shooterSpeed, shooterSpeed);
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



    public boolean atSpeed() {
        return (rpsTopSetpoint + ShooterConstants.rpsTolerance >= io.getTopVelocity() && rpsTopSetpoint - ShooterConstants.rpsTolerance <= io.getTopVelocity() && rpsBotSetpoint + ShooterConstants.rpsTolerance >= io.getBottomVelocity() && rpsBotSetpoint - ShooterConstants.rpsTolerance <= io.getBottomVelocity() );
    }

    public boolean aboveSpeed(double rpm) {
        return (io.getTopVelocity() >= rpm/60 && io.getBottomVelocity() >= rpm/60);
    }

    /**
     * 
     * @return RPM not RPS!!!! DO NOT! AND I MEAN DO NOT! CONFUSE THIS FOR RPS
     */
    public double getRPMTop() {
        return io.getTopVelocity()*60.0;
    }

    /**
     * 
     * @return RPM not RPS!!!! DO NOT! AND I MEAN DO NOT! CONFUSE THIS FOR RPS
     */
    public double getRPMBottom() {
        return io.getBottomVelocity()*60.0;
    }



}