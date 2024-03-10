package frc.robot.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.ShooterConstants;

public class BobcatUtil {
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().isEmpty() ? Alliance.Blue : DriverStation.getAlliance().get();
    }

    public static double getShooterSpeed(double spivitAngle, double ampAngle) {
        if(ampAngle >= AmpConstants.deployValue - 15){ //if the amp is within 15 degrees of being deployed, use the amp speed
            return ShooterConstants.ampShootRPMSetpoint;
        } else if (spivitAngle >= ShooterConstants.slowShooterSpivitAngle) { //if the spivit is high, we are close to the speaker, and we can use a slower setpoint
            return ShooterConstants.slowShooterRPMSetpoint;
        } else {
            return ShooterConstants.fastShooterRPMSetpoint; //otherwise use our fast setpoint
        }
    }
}
