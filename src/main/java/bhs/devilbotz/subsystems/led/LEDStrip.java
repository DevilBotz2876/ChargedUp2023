package bhs.devilbotz.subsystems.led;

import bhs.devilbotz.Robot;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase implements LEDInterface {
  public static class LEDSegmentSettings {
    private String name;
    private int startIndex;
    private int count;
    private boolean reversed;

    public LEDSegmentSettings(String name, int startIndex, int endIndex, boolean reversed) {
      this.name = name;
      this.startIndex = startIndex;
      this.count = (endIndex - startIndex) + 1;
      this.reversed = reversed;
    }
  }

  public class LEDSegment implements LEDInterface {
    public LEDSegmentSettings settings;
    private Color[] backgroundColor = new Color[0];
    private double[] backgroundEffect = LEDStrip.getEffectStatic();
    private Color[] backgroundLUT = new Color[0];

    // The following parameters control motion, if there is any
    private int backgroundMotionStartOffset = 0;
    private double backgroundMotionStartAccumulator = 0.0;
    private double backgroundMotionRate = 0;

    private Color[] overlayLUT = new Color[0];
    private int overlayMotionStartOffset = 0;
    private double overlayMotionStartAccumulator = 0.0;
    private double overlayMotionRate = 0;

    private IntegerEntry ntStartIndex;
    private IntegerEntry ntCount;
    private BooleanEntry ntReversed;
    private StringEntry ntBackgroundColor;
    private DoubleEntry ntBackgroundMotionRate;
    private StringEntry ntBackgroundEffect;
    private StringEntry ntOverlayColor;
    private DoubleEntry ntOverlayMotionRate;

    private LEDSegment(LEDSegmentSettings settings) {
      this.settings = settings;

      ntStartIndex = table.getIntegerTopic(this.settings.name + "/startIndex").getEntry(0);
      ntCount = table.getIntegerTopic(this.settings.name + "/count").getEntry(0);
      ntReversed = table.getBooleanTopic(this.settings.name + "/reversed").getEntry(false);
      ntBackgroundColor =
          table.getStringTopic(this.settings.name + "/bg/color").getEntry("Unknown");
      ntBackgroundMotionRate =
          table.getDoubleTopic(this.settings.name + "/bg/motion/rate").getEntry(0.0);
      ntBackgroundEffect =
          table.getStringTopic(this.settings.name + "/bg/effect").getEntry("Unknown");
      ntOverlayColor =
          table.getStringTopic(this.settings.name + "/overlay/color").getEntry("Unknown");
      ntOverlayMotionRate =
          table.getDoubleTopic(this.settings.name + "/overlay/motion/rate").getEntry(0.0);

      ntStartIndex.set(this.settings.startIndex);
      ntCount.set(this.settings.count);
      ntReversed.set(this.settings.reversed);
    }

    public void enable(boolean enable) {
      LEDStrip.this.enable(enable);
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
      if (true == this.settings.reversed) {
        rate = -rate;
      }

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
      if (true == this.settings.reversed) {
        rate = -rate;
      }

      overlayLUT = color;
      overlayMotionRate = rate;

      ntOverlayColor.set(color.toString());
      ntOverlayMotionRate.set(rate);
      apply();
    }

    private void apply() {
      if (0 == backgroundColor.length) return;

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
      this.updateSegment(backgroundLUT, 0, this.settings.count, backgroundMotionStartOffset);
      this.updateSegment(overlayLUT, overlayMotionStartOffset, overlayLUT.length, 0);
    }

    private boolean periodic() {
      //        rainbow();
      boolean needsUpdate = false;
      boolean backgroundNeedsUpdate = false;
      boolean overlayNeedsUpdate = false;

      if (0 != backgroundMotionRate) {
        backgroundMotionStartAccumulator += backgroundMotionRate;

        if (backgroundMotionStartOffset != (int) backgroundMotionStartAccumulator) {
          backgroundNeedsUpdate = true;
        }
      }

      if (overlayLUT.length > 0) {
        overlayMotionStartAccumulator += overlayMotionRate;

        if ((true == backgroundNeedsUpdate)
            || (overlayMotionStartOffset != (int) overlayMotionStartAccumulator)) {}
        overlayNeedsUpdate = true;
      }

      if (true == backgroundNeedsUpdate) {
        backgroundMotionStartOffset = (int) backgroundMotionStartAccumulator;

        // Apply motion effect to LED strip
        this.updateSegment(backgroundLUT, 0, this.settings.count, backgroundMotionStartOffset);
        needsUpdate = true;
        backgroundNeedsUpdate = true;
      } else if (true == overlayNeedsUpdate) {
        // We need to restore the static background since the overlay needs to be updated
        this.updateSegment(
            backgroundLUT,
            overlayMotionStartOffset,
            overlayLUT.length,
            backgroundMotionStartOffset);
        needsUpdate = true;
      }

      if (true == overlayNeedsUpdate) {
        overlayMotionStartOffset = (int) overlayMotionStartAccumulator;

        this.updateSegment(overlayLUT, overlayMotionStartOffset, overlayLUT.length, 0);
        needsUpdate = true;
      }

      return needsUpdate;
    }

    private void updateSegment(Color[] colorLUT, int startIndex, int count, int startOffset) {
      // See if we need to split the command to handle the wrap
      int logicalStartIndex = startIndex % this.settings.count;
      if (logicalStartIndex < 0) {
        logicalStartIndex += this.settings.count;
      }
      int wrapCount = (logicalStartIndex + count) - this.settings.count;

      if (wrapCount > 0) {
        // We need to split the update into two calls to handle the logical wrap
        LEDStrip.this.updateStrip(
            colorLUT, logicalStartIndex + this.settings.startIndex, count - wrapCount, startOffset);

        LEDStrip.this.updateStrip(
            colorLUT, this.settings.startIndex, wrapCount, startOffset + wrapCount);
      } else {
        LEDStrip.this.updateStrip(
            colorLUT, logicalStartIndex + this.settings.startIndex, count, startOffset);
      }
    }
  }

  private final AddressableLED strip;
  private final AddressableLEDBuffer buffer;

  //  private int rainbowFirstPixelHue;

  // Simulation Variables
  private AddressableLEDSim stripSim;

  private LEDSegment segment[];

  // Network Table Based Debug Status
  protected NetworkTableInstance inst = NetworkTableInstance.getDefault();
  protected NetworkTable table = inst.getTable("LED");

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
    this(port, count, null);
  }

  public LEDStrip(int port, int count, LEDSegmentSettings segmentSettings[]) {
    // Create LED strip object
    strip = new AddressableLED(port);

    // Create buffer containing LED values
    buffer = new AddressableLEDBuffer(count);
    strip.setLength(buffer.getLength());

    if (null == segmentSettings) {
      segmentSettings = new LEDSegmentSettings[1];
      segmentSettings[0] = new LEDSegmentSettings("Segment[0]", 0, count, false);
    }

    segment = new LEDSegment[segmentSettings.length];
    for (int i = 0; i < segmentSettings.length; i++) {
      segment[i] = new LEDSegment(segmentSettings[i]);
    }
    setupSimulation();
  }

  void setupSimulation() {
    if (Robot.isSimulation()) {
      stripSim = new AddressableLEDSim(strip);
    }
  }

  @Override
  public void periodic() {
    boolean needsUpdate = false;
    for (int i = 0; i < segment.length; i++) {
      needsUpdate |= segment[i].periodic();
    }

    if (true == needsUpdate) {
      update();
    }
  }

  public LEDSegment[] getSegments() {
    return segment;
  }

  public void enable(boolean enable) {
    if (true == enable) {
      strip.start();
    } else {
      strip.stop();
    }
  }

  public void setBackgroundColor(Color color) {
    for (int i = 0; i < segment.length; i++) {
      segment[i].setBackgroundColor(color);
    }
  }

  public void setBackgroundColor(Color[] color) {
    for (int i = 0; i < segment.length; i++) {

      segment[i].setBackgroundColor(color);
    }
  }

  public void setBackgroundEffect(LEDEffectType effectType) {
    for (int i = 0; i < segment.length; i++) {
      segment[i].setBackgroundEffect(effectType);
    }
  }

  public void setBackgroundMotionRate(double rate) {
    for (int i = 0; i < segment.length; i++) {
      segment[i].setBackgroundMotionRate(rate);
    }
  }

  public void setOverlay(Color color, int length, double rate) {
    for (int i = 0; i < segment.length; i++) {

      segment[i].setOverlay(color, length, rate);
    }
  }

  public void setOverlay(Color[] color, double rate) {
    for (int i = 0; i < segment.length; i++) {

      segment[i].setOverlay(color, rate);
    }
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
  private int getPhysicalIndex(int logicalIndex) {
    int physicalIndex = logicalIndex % buffer.getLength();

    if (physicalIndex < 0) {
      physicalIndex += buffer.getLength();
    }

    return physicalIndex;
  }

  private void updateStrip(Color[] colorLUT, int startIndex, int count, int startOffset) {
    // For every pixel
    for (int i = startIndex; i < (startIndex + count); i++) {
      int physicalIndex = getPhysicalIndex(i);
      buffer.setLED(
          physicalIndex, colorLUT[Math.abs(physicalIndex + startOffset) % colorLUT.length]);
    }
  }

  private void update() {
    strip.setData(buffer);
  }
}
