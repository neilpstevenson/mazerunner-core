#pragma once

class Indicators;
extern Indicators indicators;

class Indicators
{
  public:
    Indicators() {}

    void begin()
    {
      pinMode(LED_LEFT_IO, OUTPUT);
      digitalWrite(LED_LEFT_IO, 0);
      pinMode(LED_RIGHT_IO, OUTPUT);
      digitalWrite(LED_RIGHT_IO, 0);
   }

    void showReadyIndicator(bool on)
    {
      digitalWrite(LED_LEFT_MEZ, on);
      digitalWrite(LED_RIGHT_MEZ, on);
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
      digitalWrite(LED_MID_IO, 0);
      digitalWrite(LED_RIGHT_IO, 0);
    }
    void indicateRightTurn()
    {
      digitalWrite(LED_LEFT_IO, 0);
      digitalWrite(LED_MID_IO, 0);
      digitalWrite(LED_RIGHT_IO, 1);
    }
    void indicateForward()
    {
      digitalWrite(LED_LEFT_IO, 0);
      digitalWrite(LED_MID_IO, 1);
      digitalWrite(LED_RIGHT_IO, 0);
    }
    void indicateAboutTurn()
    {
      digitalWrite(LED_LEFT_IO, 1);
      digitalWrite(LED_MID_IO, 1);
      digitalWrite(LED_RIGHT_IO, 1);
    }
    void indicateTurnsOff()
    {
      digitalWrite(LED_LEFT_IO, 0);
      digitalWrite(LED_MID_IO, 0);
      digitalWrite(LED_RIGHT_IO, 0);
    }

};
