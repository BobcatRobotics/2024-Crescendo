package frc.robot.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BobcatUtil {
    public static Alliance getAlliance(){
        return DriverStation.getAlliance().isEmpty() ? Alliance.Blue : DriverStation.getAlliance().get();
    }
}
