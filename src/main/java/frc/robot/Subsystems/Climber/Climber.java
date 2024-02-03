package frc.robot.Subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private final ClimberIOInputsAutoLogged climberInterface = new ClimberIOInputsAutoLogged();

    ClimberIO climber;
    public Climber(ClimberIO ClimbModule){
        this.climber = ClimbModule;
    }

    double rotationAmount = Constants.climberConstants.rotationToTopAmount;

    public void deployClimber(double rotationAmount){
        rotationAmount-=climberInterface.climberMotorPosition;
        climber.run(rotationAmount);
        climber.stop();
        //Rotation amount should be how many encoder counts it takes for climber to get to the top
    }

    public void retractClimber(double rotationAmount){
        climber.inverseDirection();
        rotationAmount-=climberInterface.climberMotorPosition;
        climber.run(rotationAmount);
        climber.stop();
    }

    public void periodic(){
        climber.updateInputs(climberInterface);
    }
}
