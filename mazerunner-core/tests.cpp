/*
 * File: tests.cpp
 * Project: mazerunner
 * File Created: Tuesday, 16th March 2021 10:17:18 pm
 * Author: Peter Harrison
 * -----
 * Last Modified: Wednesday, 14th April 2021 12:59:27 pm
 * Modified By: Peter Harrison
 * -----
 * MIT License
 *
 * Copyright (c) 2021 Peter Harrison
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "tests.h"
#include "config.h"
#include "encoders.h"
#include "motion.h"
#include "motors.h"
#include "mouse.h"
#include "profile.h"
#include "reports.h"
#include "sensors.h"

//***************************************************************************//

/**
 * By turning in place through 360 degrees, it should be possible to get a
 * sensor calibration for all sensors?
 *
 * At the least, it will tell you about the range of values reported and help
 * with alignment, You should be able to see clear maxima 180 degrees apart as
 * well as the left and right values crossing when the robot is parallel to
 * walls either side.
 *
 * Use either the normal report_sensor_track() for the normalised readings
 * or report_sensor_track_raw() for the readings straight off the sensor.
 *
 * Sensor sensitivity should be set so that the peaks from raw readings do
 * not exceed about 700-800 so that there is enough headroom to cope with
 * high ambient light levels.
 *
 * @brief turn in place while streaming sensors
 */

void test_sensor_spin_calibrate() {
  enable_sensors();
  delay(100);
  reset_drive_system();
  enable_motor_controllers();
  disable_steering();
  report_sensor_track_header();
  rotation.start(360, 180, 0, 1800);
  while (not rotation.is_finished()) {
    report_sensor_track_raw();
  }
  reset_drive_system();
  disable_sensors();
  delay(100);
}

//***************************************************************************//
/**
 * Edge detection test displays the position at which an edge is found when
 * the robot is travelling down a straight.
 *
 * Start with the robot backed up to a wall.
 * Runs forward for 150mm and records the robot position when the trailing
 * edge of the adjacent wall(s) is found.
 *
 * The value is only recorded to the nearest millimeter to avoid any
 * suggestion of better accuracy than that being available.
 *
 * Note that UKMARSBOT, with its back to a wall, has its wheels 43mm from
 * the cell boundary.
 *
 * This value can be used to permit forward error correction of the robot
 * position while exploring.
 *
 * @brief find sensor wall edge detection positions
 */

void test_edge_detection() {
  bool left_edge_found = false;
  bool right_edge_found = false;
  int left_edge_position = 0;
  int right_edge_position = 0;
  int left_max = 0;
  int right_max = 0;
  enable_sensors();
  delay(100);
  reset_drive_system();
  enable_motor_controllers();
  disable_steering();
  Serial.println(F("Edge positions:"));
  forward.start(FULL_CELL - 30.0, 100, 0, 1000);
  while (not forward.is_finished()) {
    if (g_lss > left_max) {
      left_max = g_lss;
    }

    if (g_rss > right_max) {
      right_max = g_rss;
    }

    if (not left_edge_found) {
      if (g_lss < left_max / 2) {
        left_edge_position = int(0.5 + forward.position());
        left_edge_found = true;
      }
    }
    if (not right_edge_found) {
      if (g_rss < right_max / 2) {
        right_edge_position = int(0.5 + forward.position());
        right_edge_found = true;
      }
    }
    delay(5);
  }
  Serial.print(F("Left: "));
  if (left_edge_found) {
    Serial.print(BACK_WALL_TO_CENTER + left_edge_position);
  } else {
    Serial.print('-');
  }

  Serial.print(F("  Right: "));
  if (right_edge_found) {
    Serial.print(BACK_WALL_TO_CENTER + right_edge_position);
  } else {
    Serial.print('-');
  }
  Serial.println();

  reset_drive_system();
  disable_sensors();
  delay(100);
}

void test_SS90ER() {
  // enter your function call here
  // test SS90ER
  reset_drive_system();
  enable_motor_controllers();
  enable_sensors();
  float run_in = 5.0;   // mm
  float run_out = 10.0; // mm
  float angle = -90.0;  // deg
  float omega = 280;    // deg/s integer such that omega = 8 n
  float alpha = 4000;   // deg/s/s
  bool triggered = false;
  disable_steering();
  float distance = BACK_WALL_TO_CENTER + 100 + run_in;
  forward.start(distance, DEFAULT_TURN_SPEED, DEFAULT_TURN_SPEED, SEARCH_ACCELERATION);
  while (not forward.is_finished()) {
    delay(2);
  }
  Serial.print('R');
  print_justified(forward.position(), 4);
  Serial.print(' ');
  print_justified(get_front_sensor(), 3);
  Serial.println();
  rotation.start(angle, omega, 0, alpha);
  while (not rotation.is_finished()) {
    delay(2);
  }
  forward.start(run_out + 100, DEFAULT_TURN_SPEED, 0, SEARCH_ACCELERATION);
  while (not forward.is_finished()) {
    delay(2);
  }
  reset_drive_system();
}

void test_SS90EL() {
  // enter your function call here
  // test SS90EL
  reset_drive_system();
  enable_motor_controllers();
  enable_sensors();
  float run_in = 5.0;   // mm
  float run_out = 10.0; // mm
  float angle = 90.0;   // deg
  float omega = 280;    // deg/s
  float alpha = 4000;   // deg/s/s
  bool triggered = false;
  disable_steering();
  float distance = BACK_WALL_TO_CENTER + 100 + run_in;
  forward.start(distance, DEFAULT_TURN_SPEED, DEFAULT_TURN_SPEED, SEARCH_ACCELERATION);
  while (not forward.is_finished()) {
    delay(2);
  }
  Serial.print('L');
  print_justified(forward.position(), 4);
  Serial.print(' ');
  print_justified(get_front_sensor(), 3);
  Serial.println();
  rotation.start(angle, omega, 0, alpha);
  while (not rotation.is_finished()) {
    delay(2);
  }
  forward.start(100 + run_out, DEFAULT_TURN_SPEED, 0, SEARCH_ACCELERATION);
  while (not forward.is_finished()) {
    delay(2);
  }
  reset_drive_system();
}
