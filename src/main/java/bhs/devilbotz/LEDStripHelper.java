package bhs.devilbotz;

import bhs.devilbotz.lib.LEDModes;
import bhs.devilbotz.subsystems.led.LEDEffectType;
import bhs.devilbotz.subsystems.led.LEDStrip;
import edu.wpi.first.wpilibj.util.Color;

public class LEDStripHelper {
  public static void setMode(LEDStrip strip, LEDModes mode) {

    // Handle static vs animation mode assignments
    switch (mode) {
      case SET_ARM_UP:
        strip.setOverlay(Color.kWhite, 5, -0.2);
        break;
      case SET_ARM_DOWN:
        strip.setOverlay(Color.kWhite, 5, 0.2);
        break;
      case SET_ARM_IDLE:
      case CLEAR:
        strip.setOverlay(Color.kWhite, 0, 0);
        break;
      default:
        // do nothing
        break;
    }

    // update state and static colors
    switch (mode) {
      case SET_RED:
        strip.setBackgroundColor(Color.kRed);
        break;
      case SET_BLUE:
        strip.setBackgroundColor(Color.kBlue);
        break;
      case SET_CONE:
        strip.setBackgroundColor(Color.kYellow);
        break;
      case SET_CUBE:
        strip.setBackgroundColor(Color.kPurple);
        break;
      case SET_AUTONOMOUS:
        strip.setBackgroundColor(Color.kGreen);
        break;
      case CLEAR:
        strip.enable(false);
        return;
      default:
        break;
    }

    strip.setBackgroundEffect(LEDEffectType.SINUSOID);
    strip.setBackgroundMotionRate(0.1);
    strip.enable(true);
  }
}
