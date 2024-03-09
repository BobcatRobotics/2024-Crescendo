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
        if(ampAngle >= AmpConstants.deployValue - AmpConstants.deployTolerance){
            return ShooterConstants.ampShootRPMSetpoint;
        } else if (spivitAngle >= ShooterConstants.slowShooterSpivitAngle) {
            return ShooterConstants.slowShooterRPMSetpoint;
        } else {
            return ShooterConstants.fastShooterRPMSetpoint;
        }
    }
}
