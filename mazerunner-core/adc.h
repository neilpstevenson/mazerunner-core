#include "PinNames.h"
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

#ifndef ADC_H
#define ADC_H

#include <Arduino.h>
#include <wiring_private.h>
#include "config.h"
#include <mbed.h>

/***
 * The AnalogueConverter class samples a fixed number of ADC channels from 0 to
 * MAX_CHANNELS and makes the readings available to the rest of the program.
 *
 * Each channel is sampled once with the sensor emitters off and once with the emitters on.
 *
 * The first set of samples is stored in the array m_adc_dark[].
 *
 * The second set, with the emitter on, is stored in m_adc_lit[].
 *
 * for wall sensors, you should use the get_raw() method to read the (lit-dark) values.
 *
 * The class does not care what is connected to the adc channel. It just gathers readings.
 *
 * The battery monitor and analogue button channels are also converted here to avoid conflict.
 * Your config file should declare the channel number for these as well as the actual sensors.
 *
 * This class is specific to UKMARSBOT with a ATmega328p processor. If you are using any other
 * processor, you will need to re-write this class.
 *
 * To save overhead, the class works by using the conversion complete interrupt to gather
 * results only after a successful conversion. The entire sequence is begun by enabling the
 * appropriate interrupt and then starting an ADC conversion. Each interrupt action takes
 * about 5us and conversions take about 30us. Since each channel gets converted twice, the
 * entire sequence takes about 620us to complete but only uses about 100us of processor time.
 *
 * Even if the emitters are disabled, the conversions are still performed but the lit
 * values and dark values will be very similar.
 *
 * Although the code exists almost entirely in this header file, C++ requires the actual
 * instance of the class and its interrupt service routine to be somewhere in a .cpp file.
 * I have placed the instances in the main project file.
 *
 * TODO: some space and time could be saved by only storing the difference between the
 * lit and dark values but it complicates matters elsewhere if code tries to read the
 * values in the middle of the interrupt sequence. If you have all the channels read
 * in one go in a single interrupt service routine - or in systick - then that will not
 * be a problem.
 *
 * TODO: The inclusion in the class of information about the emitters is unfortunate but this
 * is the simplest scheme I could envisage. If there are two emitters, both are turned on
 * together in this version. It would be better to use different emitters for each sensor type.
 * that is left as an exercise for the reader.
 *
 * The code assumes the use of the advanced wall sensor board where there are two emitters. It
 * will work just as well with the basic wall sensor if you simply use the same pin name for
 * both emitter entries.
 *
 * TODO: The A4 and A5 channels get converted in the sequence. If you are expecting to use
 * these for an I2C device, they will need to be skipped.
 *
 * TODO: If only four sensor channels are used, there are opportunities to optimise this
 * code if you are so inclined.
 *
 * PORTING: A simulator may provide fake values as it sees fit and without the delays
 * or interrupts
 *
 *
 */

class AnalogueConverter;

extern AnalogueConverter adc;

class AnalogueConverter {
 public:
  enum {
    MAX_CHANNELS = 4,
  };

  AnalogueConverter() : m_emitter_front(PinName(EMITTER_FRONT)),
                        m_emitter_diagonal(PinName(EMITTER_DIAGONAL))
  {}

  void enable_emitters() {
    m_emitters_enabled = true;
  }

  void disable_emitters() {
    m_emitters_enabled = false;
  }

  // call this or nothing will work
  virtual void begin() {
    disable_emitters();
    //set_front_emitter_pin(EMITTER_FRONT);
    //set_side_emitter_pin(EMITTER_DIAGONAL);
    converter_init();
    m_configured = true;
  };

  //void set_front_emitter_pin(uint8_t pin) {
//    m_emitter_front = mbed::DigitalOut(PinName(pin), 0);
    //pinMode(pin, OUTPUT);
    //m_emitter_front_pin = pin;
//  };

//  void set_side_emitter_pin(uint8_t pin) {
//    m_emitter_diagonal = mbed::DigitalOut(PinName(pin), 0);
    //pinMode(pin, OUTPUT);
    //m_emitter_diagonal_pin = pin;
//  };

  //uint8_t emitter_front() {
    //return m_emitter_front_pin;
  //};
  //uint8_t emitter_diagonal() {
    //return m_emitter_diagonal_pin;
  //};

  void converter_init() {
    // Set up the HAL objects
    analogin_init(&m_halObject[0], p26);
    analogin_init(&m_halObject[1], p27);
    analogin_init(&m_halObject[2], p28);
    analogin_init(&m_halObject[3], p29);
    m_emitter_diagonal = 0;
    m_emitter_front = 0;
  }

  void start_conversion_cycle() {
    //mbed::DigitalOut emDiag(PinName(emitter_diagonal()), 0);
    //mbed::DigitalOut emFront(PinName(emitter_front()), 0);

    // Dark
    m_adc_dark[RSS_ADC_CHANNEL] = analogin_read_u16(&m_halObject[RSS_ADC_CHANNEL]) >> 4;
    m_adc_dark[LSS_ADC_CHANNEL] = analogin_read_u16(&m_halObject[LSS_ADC_CHANNEL]) >> 4;
    m_adc_dark[RFS_ADC_CHANNEL] = analogin_read_u16(&m_halObject[RFS_ADC_CHANNEL]) >> 4;
    if(RFS_ADC_CHANNEL != LFS_ADC_CHANNEL)
      m_adc_dark[LFS_ADC_CHANNEL] = analogin_read_u16(&m_halObject[LFS_ADC_CHANNEL]) >> 4;

    // Lit - front
    {ATOMIC 
    {
      if (m_emitters_enabled) {
        m_emitter_front = 1;
      }
      wait_ns(ILLUMINATION_TO_ADC_DELAY_NS);
      m_adc_lit[RFS_ADC_CHANNEL] = analogin_read_u16(&m_halObject[RFS_ADC_CHANNEL]) >> 4;
      if(RFS_ADC_CHANNEL != LFS_ADC_CHANNEL)
        m_adc_lit[LFS_ADC_CHANNEL] = analogin_read_u16(&m_halObject[LFS_ADC_CHANNEL]) >> 4;
      // Emitters off
      m_emitter_front = 0;
    }}

    // Lit - sides
    {ATOMIC 
    {
      if (m_emitters_enabled) {
        m_emitter_diagonal = 1;
      }
      wait_ns(ILLUMINATION_TO_ADC_DELAY_NS);
      m_adc_lit[RSS_ADC_CHANNEL] = analogin_read_u16(&m_halObject[RSS_ADC_CHANNEL]) >> 4;
      m_adc_lit[LSS_ADC_CHANNEL] = analogin_read_u16(&m_halObject[LSS_ADC_CHANNEL]) >> 4;
      m_emitter_diagonal = 0;
    }}

  }

  int get_lit(const int i) const {
    return m_adc_lit[i];
  }

  int get_dark(const int i) const {
    return m_adc_dark[i];
  }

  int get_raw(const int i) const {
    int diff;
    ATOMIC {
      diff = max(1, m_adc_lit[i] - m_adc_dark[i]);
    }
    return diff;
  }

 private:
  analogin_t m_halObject[MAX_CHANNELS];
  volatile int m_adc_dark[MAX_CHANNELS];
  volatile int m_adc_lit[MAX_CHANNELS];
  mbed::DigitalOut m_emitter_diagonal;
  mbed::DigitalOut m_emitter_front;
//  uint8_t m_emitter_front_pin = -1;
//  uint8_t m_emitter_diagonal_pin = -1;
  uint8_t m_index = 0;
  bool m_emitters_enabled = false;
  bool m_configured = false;
  uint8_t m_phase = 0;  // used in the isr
  uint8_t m_channel = 0;
};

#endif