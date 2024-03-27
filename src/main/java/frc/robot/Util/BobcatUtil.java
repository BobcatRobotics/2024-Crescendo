package frc.robot.Util;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.CANdleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.CANdle.BuiltInAnimations;

public class BobcatUtil {
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().isEmpty() ? Alliance.Blue : DriverStation.getAlliance().get();
    }

    public static boolean isBlue() {
        return getAlliance() == Alliance.Blue;
    }

    public static boolean isRed() {
        return getAlliance() == Alliance.Red;
    }

    public static double getShooterSpeed(double spivitAngle, double ampAngle) {
        if (ampAngle >= AmpConstants.deployValue - 15) { // if the amp is within 15 degrees of being deployed, use the
                                                         // amp speed
            return ShooterConstants.ampShootRPMSetpoint;
        } else if (spivitAngle >= ShooterConstants.slowShooterSpivitAngle) { // if the spivit is high, we are close to
                                                                             // the speaker, and we can use a slower
                                                                             // setpoint
            return ShooterConstants.slowShooterRPMSetpoint;
        } else {
            return ShooterConstants.fastShooterRPMSetpoint; // otherwise use our fast setpoint
        }
    }

    public static Animation getBuiltInAnimation(BuiltInAnimations animation) {
        switch (animation) {
            case ColorFlow:
                return new ColorFlowAnimation(128, 20, 70, 0, 0.7, CANdleConstants.LedCount, Direction.Forward);
            case Fire:
                return new FireAnimation(0.5, 0.7, CANdleConstants.LedCount, 0.7, 0.5);
            case Larson:
                return new LarsonAnimation(0, 255, 46, 0, 1, CANdleConstants.LedCount, BounceMode.Front, 3);
            case Rainbow:
                return new RainbowAnimation(1, 0.1, CANdleConstants.LedCount);
            case RgbFade:
                return new RgbFadeAnimation(0.7, 0.4, CANdleConstants.LedCount);
            case SingleFade:
                return new SingleFadeAnimation(50, 2, 200, 0, 0.5, CANdleConstants.LedCount);
            case Strobe:
                return new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, CANdleConstants.LedCount);
            case Twinkle:
                return new TwinkleAnimation(30, 70, 60, 0, 0.4, CANdleConstants.LedCount, TwinklePercent.Percent6);
            case TwinkleOff:
                return new TwinkleOffAnimation(70, 90, 175, 0, 0.8, CANdleConstants.LedCount,
                        TwinkleOffPercent.Percent100);
            default:
                return new StrobeAnimation(0, 0, 0);
        }
    }
}
