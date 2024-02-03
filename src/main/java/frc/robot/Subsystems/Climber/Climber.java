package frc.robot.Subsystems.Climber;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    ClimberIO climber;
    public Climber(ClimberIO ClimbModule){
        this.climber = ClimbModule;
    }

    public void periodic(){
        climber.updateConfigs();
    }
}
