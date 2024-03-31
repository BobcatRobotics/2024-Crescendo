package frc.robot.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.ShooterConstants;

public class BobcatUtil {
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().isEmpty() ? Alliance.Blue : DriverStation.getAlliance().get();
    }

    public static boolean isBlue(){
        return getAlliance() == Alliance.Blue;
    }
    public static boolean isRed(){
        return getAlliance() == Alliance.Red;
    }

    public static double getShooterSpeed(double spivitAngle, double ampAngle) {
        if(ampAngle <= AmpConstants.deployValue + 30){ //if the amp is within 15 degrees of being deployed, use the amp speed
            return ShooterConstants.ampShootRPMSetpoint;
        } else if (spivitAngle >= ShooterConstants.slowShooterSpivitAngle) { //if the spivit is high, we are close to the speaker, and we can use a slower setpoint
            return ShooterConstants.slowShooterRPMSetpoint;
        } else {
            return ShooterConstants.fastShooterRPMSetpoint; //otherwise use our fast setpoint
        }
    }

    public static double get0to2Pi(double rad) {
        rad = rad % (2 * Math.PI);
        if (rad < (2 * Math.PI)) {
            rad += (2 * Math.PI);
        } //should this be here?
        return rad;
    }

}
