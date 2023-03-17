package bhs.devilbotz.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

public interface LEDInterface {
  public void enable(boolean enable);

  public void setBackgroundColor(Color color);

  public void setBackgroundColor(Color[] colors);

  public void setBackgroundEffect(LEDEffectType effectType);

  public void setBackgroundMotionRate(double rate);

  public void setOverlay(Color color, int length, double rate);

  public void setOverlay(Color[] color, double rate);
}
