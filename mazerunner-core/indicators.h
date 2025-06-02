#pragma once

class Indicators;
extern Indicators indicators;

class Indicators
{
  public:
    Indicators() {}

    void begin()
    {
    }

    void showRedIndicator(bool on)
    {
       digitalWrite(LED_RIGHT_IO, on);
    }

    // Simple 3-bit RGB colour
    void showColourIndex(int colour)
    {
        digitalWrite(LED_LEFT_MEZ, (colour & 0x10));
        digitalWrite(LED_RIGHT_MEZ, (colour & 0x08));
        digitalWrite(LED_LEFT_IO, (colour & 0x04));
        digitalWrite(LED_MID_IO, (colour & 0x02));
        digitalWrite(LED_RIGHT_IO, (colour & 0x01));
    }

    // Simple 3-bit RGB colour + G for 0x08 and R for 0x10
    void showMenuIndex(int index)
    {
        showColourIndex(index);
    }

    /***
    * Visual feedback by flashing the mezzanine LED indicators
    */
    void blink(int count, int r, int g, int b) 
    {
      for (int i = 0; i < count; i++) {
        digitalWrite(LED_LEFT_MEZ, 1);
        digitalWrite(LED_RIGHT_MEZ, 1);
        delay(100);
        digitalWrite(LED_LEFT_MEZ, 0);
        digitalWrite(LED_RIGHT_MEZ, 0);
        delay(100);
      }
  }
};
