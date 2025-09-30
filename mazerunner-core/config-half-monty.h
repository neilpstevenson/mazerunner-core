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

#pragma once
#include <Arduino.h>

// hardware configuration for HALF MONTY with a Waveshare RP2040 Zero board

//**** IO CONFIGURATION ****************************************************//
const uint8_t ENCODER_LEFT_CLK = 8;
const uint8_t ENCODER_RIGHT_CLK = 3;
const uint8_t ENCODER_LEFT_B = 9;
const uint8_t ENCODER_RIGHT_B = 2;
const uint8_t LED_LEFT_IO = 10;   // Green
const uint8_t LED_RIGHT_IO = 11;  // RED
const uint8_t LED_NEOPIXEL_IO = 16;
const uint8_t MOTOR_LEFT_A = 5;
const uint8_t MOTOR_RIGHT_A = 7;
const uint8_t MOTOR_LEFT_B = 4;
const uint8_t MOTOR_RIGHT_B = 6;
const uint8_t EMITTER_A = 14; // Forward
const uint8_t EMITTER_B = 15; // Sides

// the sensor ADC channels in case we have no special use for a given channel
//const uint8_t SENSOR_0 = A0;
//const uint8_t SENSOR_1 = A1;
//const uint8_t SENSOR_2 = A2;
//const uint8_t SENSOR_3 = A3;
//const uint8_t SENSOR_4 = A4;
//const uint8_t SENSOR_5 = A5;
const uint8_t SWITCH_SELECT_PIN = 12;
const uint8_t SWITCH_GO_PIN = 13;
//const uint8_t BATTERY_PIN = A7;

// Delay between the sensor illuminator on to first ADC reading
const int ILLUMINATION_TO_ADC_DELAY_NS = 20000;

// SerialPort port
const int SERIAL_PORT_TX = 0;
const int SERIAL_PORT_RX = 1;

//#define USE_USB_SERIAL_PORT

#ifdef USE_USB_SERIAL_PORT
static UART &SerialPort = Serial;    // USB Serial
#else
static UART &SerialPort = Serial1;   // UART0 (pins 0 & 1)
#endif

/******************************************************************************
 * ATOMIC OPERATIONS for ATMEGA328 ONLY
 * Since the ATMega328 is an 8 bit processor it is possible that you will end
 * up trying to read a multi-byte quantity that is modified in an interrupt while
 * you are doing the read or write. The result is a corrupt value. 32 bit processors
 * are unlikely to suffer from this since quantities are read in a single operation.
 *
 * The AVR compiler provides a method for you to disable interrupts for the
 * duration of a block of code and then restore the state at the end of the block.
 *
 * It is not enough to simply turn off interrupts and then turn them back on because
 * you need to remember the state of the interrupt enable flag at the start of the
 * block.
 *
 * These macros do this for you and should be either modified for different processors
 * or bypassed if needed.
 *
 * Use like this:
 * ATOMIC {
 * // code to protect
 * }
 *
 */
#if defined(__AVR__)
#include <util/atomic.h>
#define ATOMIC ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#else
#define ATOMIC bool criticalSectionLockBlock = true; for(mbed::CriticalSectionLock criticalSectionLock; criticalSectionLockBlock; criticalSectionLockBlock = false)
#endif
//***************************************************************************//

/***
 * Finally, a little-known provision of the compiler lets you
 * configure the standard printf() function to print directly
 * to a SerialPort device. There is a cost overhead if you are not
 * using printf() or sprintf() elsewhere but it is a great convenience
 * if you want formatted printing to the SerialPort port.
 *
 * To use this facility add a call to redirectPrintf() early in the
 * setup() function of your code.
 *
 * TODO: this does not work with the NRF52 compiler.
 */

#if !defined(ARDUINO_ARCH_NRF52840)
// Function that printf and related will use to print
int serial_putchar(char c, FILE *f) {
  if (c == '\n') {
    // TODO do we need to add carriage returns? I think not.
    SerialPort.write('\r');
  }
  return SerialPort.write(c) == 1 ? 0 : 1;
}

//FILE serial_stdout;
void redirectPrintf() {
  // Redirect stdout so that we can use printf() with the console
  //fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  //serial_stdout = FDEV_SETUP_STREAM(serial_putchar, NULL, NULL, _FDEV_SETUP_RW);
  //stdout = &serial_stdout;
  //stdout = mbed::fdopen(SerialPort, "w+");
}
#else
void redirectPrintf(){};
#endif
