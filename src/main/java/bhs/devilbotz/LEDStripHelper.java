package bhs.devilbotz;

import bhs.devilbotz.Constants.LedConstants;
import bhs.devilbotz.lib.LEDModes;
import bhs.devilbotz.subsystems.led.LEDEffectType;
import bhs.devilbotz.subsystems.led.LEDStrip;
import bhs.devilbotz.subsystems.led.LEDStrip.LEDSegment;
import bhs.devilbotz.subsystems.led.LEDStrip.LEDSegmentSettings;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDStripHelper {
  static LEDSegmentSettings[] segmentSettings = {
    new LEDSegmentSettings("Segment 0 (Right Diagonal)", 0, 65, false),
    new LEDSegmentSettings("Segment 1 (Right Arm)", 66, 124, false),
    new LEDSegmentSettings("Segment 2 (Front Bottom)", 125, 164, true),
    new LEDSegmentSettings("Segment 3 (Left Arm)", 165, 227, true),
    new LEDSegmentSettings("Segment 4 (Left Diagonal)", 228, 299, true)
  };

  public static final LEDStrip strip =
      new LEDStrip(LedConstants.LED_PWM_PORT, LedConstants.LED_COUNT, segmentSettings);

  public static final LEDSegment[] ledSegment = strip.getSegments();

  public static void setMode(LEDModes mode) {
    // Handle static vs animation mode assignments
    switch (mode) {
      case SET_ARM_UP:
        ledSegment[1].setOverlay(Color.kWhite, 5, -0.5);
        ledSegment[3].setOverlay(Color.kWhite, 5, -0.5);
        break;
      case SET_ARM_DOWN:
        ledSegment[1].setOverlay(Color.kWhite, 5, 0.5);
        ledSegment[3].setOverlay(Color.kWhite, 5, 0.5);
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

    strip.setBackgroundEffect(LEDEffectType.SINUSOID, 0);
    ledSegment[0].setBackgroundEffect(LEDEffectType.SINUSOID, 0.1);
    ledSegment[1].setBackgroundEffect(LEDEffectType.SINUSOID, 0.1);
    ledSegment[3].setBackgroundEffect(LEDEffectType.SINUSOID, 0.1);
    ledSegment[4].setBackgroundEffect(LEDEffectType.SINUSOID, 0.1);
    strip.enable(true);
  }
}
