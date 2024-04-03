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
        if(state != currState){
            leds.animate(null); //wipe old state when setting new one
        }
        currState = state;
        
        switch (state) {
            case INTAKING: //fire animation
                leds.animate(BobcatUtil.getBuiltInAnimation(BuiltInAnimations.ColorFlow));
                break;
            case INTOOK: //green
                leds.animate(new StrobeAnimation(0, 255, 0,  0, 1, CANdleConstants.LedCount)); 
                break;
            case INTAKESTALL: //red strobe animation
                leds.animate(new StrobeAnimation(255, 0, 0,  0, 0.25, CANdleConstants.LedCount));
                break;
            case NOTEHUNTING: 
                leds.animate(BobcatUtil.getBuiltInAnimation(BuiltInAnimations.Rainbow));
                break;
            case RESETPOSE: //strobe gold
                leds.animate(new StrobeAnimation(255, 170, 0,  0, 0.25, CANdleConstants.LedCount));
                break;
            case RESETGYRO: //strobe gold
                leds.animate(new StrobeAnimation(255, 170, 0, 0, 0.25, CANdleConstants.LedCount));
                break;
            case OUTAKE: //strobe red
                leds.animate(new StrobeAnimation(255, 0, 0, 0, 0.75, CANdleConstants.LedCount));
                break;
            case FEED:
                leds.animate(new StrobeAnimation(0, 255, 0, 0, 0.6, CANdleConstants.LedCount));
            case OFF:
                leds.animate(null);
                break;
            default:
                leds.animate(null);
                break;
        }
    }


    
    
}