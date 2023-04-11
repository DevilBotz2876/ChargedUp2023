#include <WS2812FX.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define PIN_NEO_PIXEL  4   // Arduino pin that connects to NeoPixel
#define NUM_PIXELS     300  // The number of LEDs (pixels) on NeoPixel

int DELAY_INTERVAL = 1;

WS2812FX ws2812fx = WS2812FX(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);

boolean signalReceived = false;

void setup() {
  ws2812fx.init();
  clear_leds();
  Serial.begin(9600);
  setLoading();
  
  ws2812fx.start();
}

void loop() {
  ws2812fx.service();

  if (Serial.available()){
    byte value = Serial.read();

    switch (value){
      case 0x0:
        clear_leds();
        break;
      case 0x1:
        setRed();
        break;
      case 0x2:
        setBlue();
        break;
      case 0x3:
        setCone();
        break;
      case 0x4:
        setCube();
        break;
      case 0x5:
        setAutonomous();
        break;
      case 0x6:
        armUp();
        break;
      case 0x7:
        armDown();
        break;
      case 0x8:
        armIdle();
        break;
      case 0x9:
        setLoading();
        break;
      case 0xA:
        voltageWarning();
        break;

      ws2812fx.start();
    }
  }


}

void clear_leds(){
  ws2812fx.clear();
}

void setCone() {
  ws2812fx.setBrightness(100);
  ws2812fx.setSegment(0, 0, NUM_PIXELS, 18, COLORS(0xFF5100, BLACK), 10, false);
}

void setRed(){
  ws2812fx.setBrightness(100);
  ws2812fx.setSegment(0, 0, NUM_PIXELS, 18, COLORS(RED, BLACK), 10, false);
}

void setBlue() {
  ws2812fx.setBrightness(100);
  ws2812fx.setSegment(0, 0, NUM_PIXELS, 18, COLORS(BLUE, BLACK), 10, false);
}

void setCube() {
  ws2812fx.setBrightness(100);
  ws2812fx.setSegment(0, 0, NUM_PIXELS, 18, COLORS(0xbf00ff, BLACK), 10, false);
}

void setLoading(){
  ws2812fx.setBrightness(100);
  ws2812fx.setSegment(0, 0, NUM_PIXELS, 55, COLORS(RED, BLUE), 50, false);
}

void setAutonomous(){
  ws2812fx.setBrightness(100);
  ws2812fx.setSegment(0, 0, NUM_PIXELS, 18, COLORS(GREEN), 1000, false);
}

void armUp(){
  ws2812fx.setBrightness(100);
  uint8_t notReversed = SIZE_MEDIUM;
  uint8_t reversed = REVERSE + SIZE_MEDIUM;

  ws2812fx.setSegment(1, 159, 223, 31, 0x45e6ff, 25, notReversed);
  ws2812fx.setSegment(2, 65, 123, 31, 0x45e6ff, 25, reversed);
}

void armDown(){
  ws2812fx.setBrightness(100);
  uint8_t notReversed = SIZE_MEDIUM;
  uint8_t reversed = REVERSE + SIZE_MEDIUM;

  ws2812fx.setSegment(1, 159, 223, 31, 0x45e6ff, 25, reversed);
  ws2812fx.setSegment(2, 65, 123, 31, 0x45e6ff, 25, notReversed);
}

void armIdle(){
  ws2812fx.removeActiveSegment(1);
  ws2812fx.removeActiveSegment(2);
}

void voltageWarning(){
  ws2812fx.setBrightness(100);
  ws2812fx.setSegment(0, 0, NUM_PIXELS, 1, 0xff0000, 2000, false);
}
