package bhs.devilbotz.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

public interface LEDInterface {
  /**
   * Sets the background color of the LED segment to a single color
   *
   * @param color the color
   */
  public void setBackgroundColor(Color color);

  /**
   * Sets the background color of the LED segment to the requested array of colors. The array is
   * repeated as needed to fill the entire segment
   *
   * @param color the array of color(s)
   */
  public void setBackgroundColor(Color[] color);

  /**
   * Sets the effect to apply to the background color with optional motion
   *
   * @param effectType the effect type to use
   * @param rate the rate at which he effect should move. rate=0 means the effect is static
   */
  public void setBackgroundEffect(LEDEffectType effectType, double rate);

  /**
   * Sets overlay to a single color of specified length with optional motion
   *
   * @param color the color
   * @param length the number of pixels of the overlay
   * @param rate the rate at which he overlay should move. rate=0 means the overlay is static
   */
  public void setOverlay(Color color, int length, double rate);

  /**
   * Sets overlay to the requested array of pixels with optional motion
   *
   * @param color the array of color(s)
   * @param rate the rate at which he overlay should move. rate=0 means the overlay is static
   */
  public void setOverlay(Color[] color, double rate);
}
