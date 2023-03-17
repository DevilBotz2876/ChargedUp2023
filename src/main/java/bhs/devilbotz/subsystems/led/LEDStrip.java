package bhs.devilbotz.subsystems.led;

import bhs.devilbotz.Robot;
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
  private class LEDSegment implements LEDInterface
  {
    private String name;
    private int startIndex;
    private int count;

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
    private StringEntry ntBackgroundColor;
    private DoubleEntry ntBackgroundMotionRate;
    private StringEntry ntBackgroundEffect;
    private StringEntry ntOverlayColor;
    private DoubleEntry ntOverlayMotionRate;
  
    private LEDSegment(String name, int startIndex, int count)
    {
      this.name = name;
      this.startIndex = startIndex;
      this.count = count;

      ntStartIndex = table.getIntegerTopic(this.name+"/startIndex").getEntry(0);
      ntCount = table.getIntegerTopic(this.name+"/count").getEntry(0);
      ntBackgroundColor = table.getStringTopic(this.name+"/bg/color").getEntry("Unknown");
      ntBackgroundMotionRate = table.getDoubleTopic(this.name+"/bg/motion/rate").getEntry(0.0);
      ntBackgroundEffect = table.getStringTopic(this.name+"/bg/effect").getEntry("Unknown");
      ntOverlayColor = table.getStringTopic(this.name+"/overlay/color").getEntry("Unknown");
      ntOverlayMotionRate =
          table.getDoubleTopic(this.name+"/overlay/motion/rate").getEntry(0.0);

      ntStartIndex.set(this.startIndex);
      ntCount.set(this.count);
    }

    public void enable(boolean enable)
    {
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
      updateSegment(backgroundLUT, 0, count, backgroundMotionStartOffset);
      updateSegment(overlayLUT, overlayMotionStartOffset, overlayLUT.length, 0);
    }

    private void updateSegment(Color[] colorLUT, int startIndex, int count, int startOffset)
    {
      LEDStrip.this.updateSegment(colorLUT, startIndex + this.startIndex, count, startOffset);
    }

    public boolean periodic() {
      //        rainbow();
      boolean needsUpdate = false;
      boolean backgroundUpdated = false;
      if (backgroundMotionRate > 0) {
        backgroundMotionStartAccumulator += backgroundMotionRate;
  
        if (backgroundMotionStartOffset != (int) backgroundMotionStartAccumulator) {
          backgroundMotionStartOffset = (int) backgroundMotionStartAccumulator;
  
          // Apply motion effect to LED strip
          updateSegment(backgroundLUT, 0, count, backgroundMotionStartOffset);
          needsUpdate = true;
          backgroundUpdated = true;
        }
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
  
      return needsUpdate;
    }  
  }

  private final AddressableLED strip;
  private final AddressableLEDBuffer buffer;

  //  private int rainbowFirstPixelHue;

  // Simulation Variables
  private AddressableLEDSim stripSim;

  LEDSegment segment[];

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
    // Create LED strip object
    strip = new AddressableLED(port);

    // Create buffer containing LED values
    buffer = new AddressableLEDBuffer(count);
    strip.setLength(buffer.getLength());

    segment = new LEDSegment[1];
    segment[0] = new LEDSegment("Segment[0]", 0, count);

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
    for (int i = 0; i < segment.length; i++)
    {
      needsUpdate |= segment[i].periodic();
    }

    if (true == needsUpdate) {
      update();
    }
  }

  public void enable(boolean enable) {
    if (true == enable) {
      strip.start();
    } else {
      strip.stop();
    }
  }

  public void setBackgroundColor(Color color)
  {
    segment[0].setBackgroundColor(color);
  }

  public void setBackgroundColor(Color[] color)
  {
    segment[0].setBackgroundColor(color);
  }

  public void setBackgroundEffect(LEDEffectType effectType)
  {
    segment[0].setBackgroundEffect(effectType);
  }

  public void setBackgroundMotionRate(double rate)
  {
    segment[0].setBackgroundMotionRate(rate);
  }

  public void setOverlay(Color color, int length, double rate)
  {
    segment[0].setOverlay(color, length, rate);
  }

  public void setOverlay(Color[] color, double rate)
  {
    segment[0].setOverlay(color, rate);    
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
