#include "ThisThread.h"
#include "cmsis_os2.h"
/******************************************************************************
 * Project: mazerunner-core                                                   *
 * -----                                                                      *
 * Copyright 2022 - 2023 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#ifndef SYSTICK_H
#define SYSTICK_H
#include <mbed.h>
#include "Arduino.h"
#include "adc.h"
#include "config.h"
#include "motion.h"
#include "motors.h"
#include "sensors.h"
#include "switches.h"
using namespace std::chrono;

class Systick {
 private:
  rtos::Thread tickerThread;

 public:
  Systick() : tickerThread(osPriorityRealtime)
  {}

  // don't let this start firing up before we are ready.
  // call the begin method explicitly.
  void begin() {
    // Start a thread that will run on each tick
    tickerThread.start({this, &Systick::tickerRun});
    delay(40);  // make sure it runs for a few cycles before we continue
  }
  /***
   * This is the SYSTICK ISR. It runs at 500Hz by default and is
   * called from the TIMER 2 interrupt (vector 7).
   *
   * All the time-critical control functions happen in here.
   *
   * interrupts are enabled at the start of the ISR so that encoder
   * counts are not lost.
   *
   * The last thing it does is to start the sensor reads so that they
   * will be ready to use next time around.
   *
   * Timing tests indicate that, with the robot at rest, the systick ISR
   * consumes about 10% of the available system bandwidth.
   *
   * With just a single profile active and moving, that increases to nearly 30%.
   * Two such active profiles increases it to about 35-40%.
   *
   * The reason that two profiles does not take up twice as much time is that
   * an active profile has a processing overhead even if there is no motion.
   *
   * Most of the load is due to that overhead. While the profile generates actual
   * motion, there is an additional load.
   *
   *
   */
  static void update() {
    // digitalWriteFast(LED_BUILTIN, 1);
    // NOTE - the code here seems to get inlined and so the function is 2800 bytes!
    // grab the encoder values first because they will continue to change
    encoders.update();
    motion.update();
    sensors.update();
    battery.update();

    motors.update_controllers(motion.velocity(), motion.omega(), sensors.get_steering_feedback());
    adc.start_conversion_cycle();
    // NOTE: no code should follow this line;
  }

  void tickerRun() {
    while(1)
    {
      rtos::ThisThread::sleep_for(2ms);
      update();
    }
  }
};

extern Systick systick;

#endif