#pragma once
#include <Adafruit_NeoPixel.h>

class Indicators;
extern Indicators indicators;

class Indicators
{
  public:
    Indicators() : 
      pixels(1, LED_NEOPIXEL_IO, NEO_GRB + NEO_KHZ800) {}

    void begin()
    {
      pixels.clear(); // Set all neopixels to 'off'
      pixels.show();

      pinMode(LED_LEFT_IO, OUTPUT);
      digitalWrite(LED_LEFT_IO, 0);
      pinMode(LED_RIGHT_IO, OUTPUT);
      digitalWrite(LED_RIGHT_IO, 0);
   }

    void showReadyIndicator(bool on)
    {
      showColourIndex(on? 0x1f : 0);
    }

    // Simple 3-bit RGB colour
    void showColourIndex(int colour)
    {
        pixels.setPixelColor(0, pixels.Color((colour & 4) ? 8 : 0, (colour & 2) ? 8 : 0, (colour & 1) ? 8 : 0));
        pixels.show();
        digitalWrite(LED_LEFT_IO, (colour & 0x08));
        digitalWrite(LED_RIGHT_IO, (colour & 0x10));
    }

    // Simple 3-bit RGB colour + G for 0x08 and R for 0x10
    void showMenuIndex(int index)
    {
        showColourIndex(index);
    }

    /***
    * Visual feedback by flashing the mezzanine and sensor LED indicators
    */
    void blink(int count, int r, int g, int b) 
    {
      for (int i = 0; i < count; i++) {
        showColourIndex(0x1f);
        delay(100);
        showColourIndex(0);
        delay(100);
      }
    }

    /**
    * Feedback on turn directions during maze runs & searches
    */
    void indicateLeftTurn()
    {
      digitalWrite(LED_LEFT_IO, 1);
      digitalWrite(LED_RIGHT_IO, 0);
    }
    void indicateRightTurn()
    {
      digitalWrite(LED_LEFT_IO, 0);
      digitalWrite(LED_RIGHT_IO, 1);
    }
    void indicateForward()
    {
      digitalWrite(LED_LEFT_IO, 0);
      digitalWrite(LED_RIGHT_IO, 0);
    }
    void indicateAboutTurn()
    {
      digitalWrite(LED_LEFT_IO, 1);
      digitalWrite(LED_RIGHT_IO, 1);
    }
    void indicateTurnsOff()
    {
      digitalWrite(LED_LEFT_IO, 0);
      digitalWrite(LED_RIGHT_IO, 0);
    }

  private:
    Adafruit_NeoPixel pixels;
};
