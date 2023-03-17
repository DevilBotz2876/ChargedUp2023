package bhs.devilbotz.subsystems.led;

import bhs.devilbotz.Robot;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase implements LEDInterface {
  private final AddressableLED strip;
  private final AddressableLEDBuffer buffer;

  //  private int rainbowFirstPixelHue;

  private Color[] backgroundColor = new Color[0];
  private double[] backgroundEffect = getEffectStatic();
  private Color[] backgroundLUT = new Color[0];

  // The following parameters control motion, if there is any
  private int backgroundMotionStartOffset = 0;
  private double backgroundMotionStartAccumulator = 0.0;
  private double backgroundMotionRate = 0;

  private Color[] overlayLUT = new Color[0];
  private int overlayMotionStartOffset = 0;
  private double overlayMotionStartAccumulator = 0.0;
  private double overlayMotionRate = 0;

  // Simulation Variables
  private AddressableLEDSim stripSim;

  // Network Table Based Debug Status
  protected NetworkTableInstance inst = NetworkTableInstance.getDefault();
  protected NetworkTable table = inst.getTable("LED");

  private StringEntry ntBackgroundColor = table.getStringTopic("bg/color").getEntry("Unknown");
  private DoubleEntry ntBackgroundMotionRate = table.getDoubleTopic("bg/motion/rate").getEntry(0.0);
  private StringEntry ntBackgroundEffect = table.getStringTopic("bg/effect").getEntry("Unknown");
  private StringEntry ntOverlayColor = table.getStringTopic("overlay/color").getEntry("Unknown");
  private DoubleEntry ntOverlayMotionRate =
      table.getDoubleTopic("overlay/motion/rate").getEntry(0.0);

  private static double[] getEffectSine(int period) {
    double effect[] = new double[period];

    for (int i = 0; i < period; i++) {
      effect[i] = Math.sin(2 * Math.PI * ((double) i / period)) / 2 + 0.5;
    }

    return effect;
  }

  private static double[] getEffectStatic() {
    double effect[] = new double[1];
    effect[0] = 1;
    return effect;
  }

  public LEDStrip(int port, int count) {
    // Create LED strip object
    strip = new AddressableLED(port);

    // Create buffer containing LED values
    buffer = new AddressableLEDBuffer(count);
    strip.setLength(buffer.getLength());

    setupSimulation();
  }

  void setupSimulation() {
    if (Robot.isSimulation()) {
      stripSim = new AddressableLEDSim(strip);
    }
  }

  @Override
  public void periodic() {
    //        rainbow();
    boolean needsUpdate = false;
    boolean backgroundUpdated = false;
    if (backgroundMotionRate > 0) {
      backgroundMotionStartAccumulator += backgroundMotionRate;

      if (backgroundMotionStartOffset != (int) backgroundMotionStartAccumulator) {
        backgroundMotionStartOffset = (int) backgroundMotionStartAccumulator;

        // Apply motion effect to LED strip
        updateSegment(backgroundLUT, 0, buffer.getLength(), backgroundMotionStartOffset);
        needsUpdate = true;
        backgroundUpdated = true;
      }

      update();
    }

    if (overlayLUT.length > 0) {
      overlayMotionStartAccumulator += overlayMotionRate;

      if ((true == backgroundUpdated)
          || (overlayMotionStartOffset != (int) overlayMotionStartAccumulator)) {
        overlayMotionStartOffset = (int) overlayMotionStartAccumulator;

        updateSegment(overlayLUT, overlayMotionStartOffset, overlayLUT.length, 0);
        needsUpdate = true;
        backgroundUpdated = false;
      }
    }

    if (true == needsUpdate) {
      update();
    }
  }

  private void apply() {
    backgroundLUT = new Color[Math.max(backgroundEffect.length, backgroundColor.length)];

    /* Initialize color LUT using transform */
    for (int i = 0; i < backgroundLUT.length; i++) {
      backgroundLUT[i] =
          new Color(
              backgroundColor[i % backgroundColor.length].red
                  * backgroundEffect[i % backgroundEffect.length],
              backgroundColor[i % backgroundColor.length].green
                  * backgroundEffect[i % backgroundEffect.length],
              backgroundColor[i % backgroundColor.length].blue
                  * backgroundEffect[i % backgroundEffect.length]);
    }
    updateSegment(backgroundLUT, 0, buffer.getLength(), backgroundMotionStartOffset);
    updateSegment(overlayLUT, overlayMotionStartOffset, overlayLUT.length, 0);
  }

  public void enable(boolean enable) {
    if (true == enable) {
      strip.start();
    } else {
      strip.stop();
    }
  }

  public void setBackgroundColor(Color color) {
    Color[] colors = new Color[1];
    colors[0] = color;
    setBackgroundColor(colors);
  }

  public void setBackgroundColor(Color[] color) {
    this.backgroundColor = color;
    ntBackgroundColor.set(color.toString());
    apply();
  }

  public void setBackgroundEffect(LEDEffectType effectType) {
    switch (effectType) {
      case STATIC:
        backgroundEffect = getEffectStatic();
        ntBackgroundEffect.set("Static");
        break;
      case SINUSOID:
        backgroundEffect = getEffectSine(10);
        ntBackgroundEffect.set("Sine");
        break;
      default:
        break;
    }
    apply();
  }

  public void setBackgroundMotionRate(double rate) {
    backgroundMotionRate = rate;
    ntBackgroundMotionRate.set(backgroundMotionRate);
  }

  public void setOverlay(Color color, int length, double rate) {
    Color[] overlayColors = new Color[length];
    for (int i = 0; i < length; i++) {
      overlayColors[i] = color;
    }
    setOverlay(overlayColors, rate);
  }

  public void setOverlay(Color[] color, double rate) {
    overlayLUT = color;
    overlayMotionRate = rate;

    ntOverlayColor.set(color.toString());
    ntOverlayMotionRate.set(rate);
    apply();
  }

  /*
    private void rainbow() {
      // For every pixel
      for (int i = 0; i < buffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        int hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
        // Set the value
        buffer.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      rainbowFirstPixelHue += 3;
      // Check bounds
      rainbowFirstPixelHue %= 180;
      strip.setData(buffer);
    }
  */
  private void updateSegment(Color[] colorLUT, int startIndex, int count, int startOffset) {
    // For every pixel
    for (int i = startIndex; i < (startIndex + count); i++) {
      int physicalIndex = (i % buffer.getLength());
      if (physicalIndex < 0) {
        physicalIndex += buffer.getLength();
      }
      buffer.setLED(
          physicalIndex, colorLUT[Math.abs(physicalIndex + startOffset) % colorLUT.length]);
    }
  }

  private void update() {
    strip.setData(buffer);
  }
}
