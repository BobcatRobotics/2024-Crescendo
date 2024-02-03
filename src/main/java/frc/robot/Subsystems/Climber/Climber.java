package frc.robot.Subsystems.Climber;

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
