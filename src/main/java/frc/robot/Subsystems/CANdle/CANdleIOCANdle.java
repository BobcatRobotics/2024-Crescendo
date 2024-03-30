package frc.robot.Subsystems.CANdle;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;

import frc.robot.Constants.CANdleConstants;
import frc.robot.Util.BobcatUtil;

public class CANdleIOCANdle implements CANdleIO {
    
    private CANdle leds;
    private CANdleState currState = CANdleState.OFF;
    public CANdleIOCANdle(){
        leds = new CANdle(CANdleConstants.CANdleID);
    }

    public void updateInputs(CANdleIOInputs inputs){
        inputs.state = currState;
    }

    @Override
    public void setLEDs(CANdleState state){
        currState = state;
        switch (state) {
            case INTAKING: //fire animation
                leds.animate(BobcatUtil.getBuiltInAnimation(BuiltInAnimations.Fire));
                break;
            case INTOOK: //green
                leds.setLEDs(0, 255, 0);
                break;
            case INTAKESTALL: //red strobe animation
                leds.animate(new StrobeAnimation(255, 0, 0));
                break;
            case NOTEHUNTING: 
                leds.animate(BobcatUtil.getBuiltInAnimation(BuiltInAnimations.Rainbow));
                break;
            case RESETPOSE: //strobe gold
                leds.animate(new StrobeAnimation(255, 170, 0));
                break;
            case RESETGYRO: //strobe gold
                leds.animate(new StrobeAnimation(255, 170, 0));
                break;
            case OUTAKE: //fast strobe orange
                leds.animate(new StrobeAnimation(255, 140, 0, 255, 1, CANdleConstants.LedCount));
                break;
            case OFF:
                leds.setLEDs(0, 0, 0);
                break;
            default:
                leds.setLEDs(0, 0, 0);
                break;
        }
    }


    
    
}