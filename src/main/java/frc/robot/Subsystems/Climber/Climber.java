package frc.robot.Subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    ClimberIO climber;
    public Climber(ClimberIO ClimbModule){
        this.climber = ClimbModule;
    }

    double rotationAmount = Constants.climberConstants.rotationToTopAmount;

    public void deployClimber(double rotationAmount){
        climber.run(rotationAmount);
        climber.stop();
        //Rotation amount should be how many encoder counts it takes for climber to get to the top
    }

    public void retractClimber(double rotationAmount){
        climber.inverseDirection();
        climber.run(rotationAmount);
        climber.stop();
    }

    public void periodic(){
        climber.updateInputs();
    }
}
