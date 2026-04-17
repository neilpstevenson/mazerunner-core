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

#ifndef MOUSE_H
#define MOUSE_H
#include "Arduino.h"
#include "config.h"
#include "maze.h"
#include "motion.h"
#include "reporting.h"
#include "sensors.h"
#include "switches.h"

/***
 * The Mouse class is really a subclass of a more generic robot. It should
 * add functionality to a robot class by providing specific mapping and
 * planning features that are more closely aligned with its purpose.
 *
 * In the same way, a ine follower and a sumo robot are also subclasses
 * of Robot.
 *
 * This code combines Robot with Mouse. If you are writing for several
 * different events, consider creating a Robot class for all the basic
 * functionality and then extending it for the individual contest events.
 */
class Mouse;
extern Mouse mouse;

class Mouse {
 public:
  enum State { FRESH_START, SEARCHING, INPLACE_RUN, SMOOTH_RUN, FINISHED };

  enum TurnType {
    SS90EL = 0,
    SS90ER = 1,
    SS90L = 2,
    SS90R = 3,
  };

  Mouse() {
    init();
  }

  void init() {
    m_handStart = false;
    sensors.set_steering_mode(STEERING_OFF);
    m_location = Location(0, 0);
    m_heading = NORTH;
  }

  //***************************************************************************//
  /**
   * change the mouse heading but do not physically turn
   */

  void set_heading(Heading new_heading) {
    m_heading = new_heading;
  }

  //***************************************************************************//
  /**
   * Used to bring the mouse to a halt, centered in a cell.
   *
   * If there is a wall ahead, it will use that for a reference to make sure it
   * is well positioned.
   *
   * On entry the robot is at some position that is an offset from the threshold
   * of the previous cell. Remember that all offset positions start at zero for the
   * cell threshold with 90mm (HALF_CELL) being the centre.
   * The robot is moving forwards. Since this is generally used in the search, the
   * position is likely to have some value between 170 and 190mm and the offset of
   * centre of the next cell is (FULL_CELL + HALF_CELL) = 270mm in the classic
   * contest.
   *
   * TODO: the critical values are robot-dependent.
   *
   * TODO: need a function just to adjust forward position
   *
   * TODO: It would be better to use the distance rather than sensor readigs here
   */
   /*
  static void stopAndAdjust() {
    float remaining = (FULL_CELL + HALF_CELL) - motion.position();
    sensors.set_steering_mode(STEERING_OFF);
    // Keep moving with the intent of stopping at the cell centre
    motion.start_move(remaining, motion.velocity(), 0, motion.acceleration());
    // While waiting, check to see if a front wall becomes visible and break
    // out early if it does
    while (not motion.move_finished()) {
      if (sensors.get_front_sum() > (FRONT_REFERENCE - 150)) {
        break;
      }
      delay(2);
    }
    // If the wait finished early because of a wall ahead then use that
    // wall to creep up on the cell centre
    if (sensors.see_front_wall) {
      while (sensors.get_front_sum() < FRONT_REFERENCE) {
        motion.start_move(10, 50, 0, 1000);
        delay(2);
      }
    }
  }
*/
  //***************************************************************************//
  /**
   * These convenience functions will bring the robot to a halt
   * before actually turning.
   *
   * Note that they do not change the robot's heading. That is
   * the responsibility of the caller.
   */

  void turn_IP180() {
    static int direction = 1;
    direction *= -1;  // alternate direction each time it is called
    motion.spin_turn(direction * 180, speed_parameters->spin_omega, speed_parameters->spin_alpha);
  }

  void turn_IP90R() {
    motion.spin_turn(-90, speed_parameters->spin_omega, speed_parameters->spin_alpha);
  }

  void turn_IP90L() {
    motion.spin_turn(90, speed_parameters->spin_omega, speed_parameters->spin_alpha);
  }

  //***************************************************************************//
  /** Search turns
   *
   * These turns assume that the robot is crossing the cell boundary but is still
   * short of the start position of the turn.
   *
   * The turn will be a smooth, coordinated turn that should finish short of
   * the next cell boundary.
   *
   * Does NOT update the mouse heading but it possibly should
   *
   *
   * TODO: There is only just enough space to get down to turn speed. Increase turn speed?
   *
   */
  void turn_smooth(int turn_id) {
    TurnParameters params = turn_params[turn_id];

    ATOMIC {
      motion.set_target_velocity(params.speed);
      //forward.set_speed((params.speed + forward.speed())/2);    // Brutal slow down!
      //forward.set_speed(params.speed);    // Brutal slow down!
    }

    float trigger = params.trigger;
    if (sensors.see_left_wall) {
      trigger += EXTRA_WALL_ADJUST;
    }
    if (sensors.see_right_wall) {
      trigger += EXTRA_WALL_ADJUST;
    }

    bool triggered_by_sensor = false;
    float turn_point = FULL_CELL + HALF_CELL - params.entry_offset;
    // Now we wait until we detect the turn start condition
    // TODO: there is room for improvement here
    //SerialPort.println(F("SMOOTH: "));
    //SerialPort.println(turn_point);SerialPort.print(" ");SerialPort.println(motion.position());
    while (motion.position() < turn_point) {
      //SerialPort.print(motion.position()); SerialPort.print(" ");
      //SerialPort.println(sensors.get_front_sum());
      if (sensors.get_front_sum() > trigger) {
        motion.set_target_velocity(motion.velocity());  // prevent speed changes
        triggered_by_sensor = true;
        break;
      }
    }
    char note = triggered_by_sensor ? 's' : 'd';
    char dir = (turn_id & 1) ? 'R' : 'L';
    reporter.log_action_status(dir, note, m_location, m_heading);  // the sensors triggered the turn
    // finally we get to actually turn
    sensors.set_steering_mode(STEERING_OFF);
    motion.turn(params.angle, params.omega, 0, params.alpha);
    // robot should be at output offset - run to the sensing position
    int end_point = HALF_CELL + params.exit_offset;
    sensors.set_steering_mode(STEER_NORMAL);
    motion.move(SENSING_POSITION - end_point, motion.velocity(), params.speed, speed_parameters->acceleration);
    motion.set_position(SENSING_POSITION);
  }

  //***************************************************************************//
  /***
   * bring the mouse to a halt in the center of the current cell. That is,
   * the cell it is entering.
   *
   * On entry, we assume that the mouse knows its position in terms of the
   * distance from the start of the last cell.
   */
  void stop_at_center() {
    bool has_wall = sensors.see_front_wall;
    if(has_wall)
    {
      // Side sensors unreliable this close to wall
      //sensors.set_steering_mode(STEERING_OFF);
      // Move to within 10mm of expected position and then 
      // crawl remainder until we're at the required position
      #define APPROACH_SPEED 30
      #define APPROACH_CHECK_DIST 5
      float remaining, velocity, acceleration;
      ATOMIC {
        remaining = (FULL_CELL + HALF_CELL) - motion.position() - APPROACH_CHECK_DIST;
        velocity = motion.velocity();
        acceleration = motion.acceleration();
        motion.start_move(remaining, velocity, APPROACH_SPEED, acceleration);
      }
#ifdef DEBUG_LOGGING
      SerialPort.print(remaining);  SerialPort.print(" ");
      SerialPort.print(velocity);SerialPort.print(" ");
      SerialPort.print(APPROACH_SPEED);SerialPort.print(" ");
      SerialPort.print(acceleration);SerialPort.print(" ");
      SerialPort.println();
#endif
      while (sensors.get_front_sum() < FRONT_REFERENCE) {
        //printf("wall dist: %d\n", sensors.get_front_sum());
#ifdef DEBUG_LOGGING
        SerialPort.println();
        reporter.report_profile();
        //reporter.print_wall_sensors();
        reporter.log_action_status('C', 'S', m_location, m_heading);
#endif
        delay(1);
      }
      // Side sensors unreliable this close to wall
      sensors.set_steering_mode(STEERING_OFF);
      //printf("stop dist: %d\n", sensors.get_front_sum());
      // Be sure robot has come to a halt.
      {ATOMIC {
        motion.start_move(5, motion.velocity(), 0, motion.acceleration());
      }
      while (!motion.move_finished()) {
        //printf("wall dist: %d\n", sensors.get_front_sum());
#ifdef DEBUG_LOGGING
        SerialPort.println();
        reporter.report_profile();
        //reporter.print_wall_sensors();
        reporter.log_action_status('C', 'F', m_location, m_heading);
#endif
        delay(1);
      }}
    }
    else
    {
      sensors.set_steering_mode(STEER_NORMAL);
      float remaining = (FULL_CELL + HALF_CELL) - motion.position();
      motion.move(remaining, motion.velocity(), 0, motion.acceleration());
    }

    // Be sure robot has come to a halt.
    motion.stop_move();
    motors.reset_controllers();

#ifdef DEBUG_LOGGING_OFF
    for(int d = 0; d < 100; d++)
    {
      delay(10);
      SerialPort.println();
      reporter.report_profile();
      //reporter.print_wall_sensors();
      reporter.log_action_status('C', 'D', m_location, m_heading);
    }
#endif
  }

  //***************************************************************************//
  /**
   * The robot is already moving so it is enough to let it carry on until
   * the next sensing position is reached.
   * Subtracting one full cell from the current position tricks the motion
   * control into thinking it is at (or just before) the start of a new cell.
   * Then it just waits until it gets to the next sensing position.
   */
  void move_ahead() {
    // How many cells can we move?
    int cell_count = maze.count_cells_ahead(m_location, m_heading);
    //SerialPort.println();
    //SerialPort.print("Cells ahead: ");
    //SerialPort.println(cell_count);

    ATOMIC {
      float current_position = motion.position();
      #define MOVE_AHEAD_DECEL_EXTRA 10
      motion.start_move(cell_count * FULL_CELL - (current_position - FULL_CELL) - MOVE_AHEAD_DECEL_EXTRA, 
                        cell_count > 1 ? speed_parameters->forward_speed_2 : speed_parameters->forward_speed, 
                        speed_parameters->forward_speed, speed_parameters->acceleration);
      // Offset by overrun distance to arrive back at the sensing position travelling at the normal speed
      motion.adjust_forward_position(current_position - FULL_CELL);
    }
    //motion.set_target_velocity(speed_parameters->forward_speed);
    //motion.adjust_forward_position(-FULL_CELL);
    for(int cell = 0; cell < cell_count; cell++)
    {
      motion.wait_until_position(SENSING_POSITION + cell*FULL_CELL);
      //SerialPort.print("Reached cell: ");
      //SerialPort.println(cell+1);
      // Don't move ahead on the last cell - will automatically do that in the main loop
      if(cell < cell_count-1)
      {
        m_location = m_location.neighbour(m_heading);
      }
    }
    motion.adjust_forward_position(-FULL_CELL * (cell_count-1));
  }

  //***************************************************************************//
  void turn_left() {
     if(speed_parameters->smooth_turns)
    {
      turn_smooth(SS90EL);
      m_heading = left_from(m_heading);
    }
    else
    {
      turn_left_ip();
    }
  }

  //***************************************************************************//
  void turn_right() {
    if(speed_parameters->smooth_turns)
    {
      turn_smooth(SS90ER);
      m_heading = right_from(m_heading);
    }
    else
    {
      turn_right_ip();
    }
  }


//***************************************************************************//
  void turn_left_ip() {
    stop_at_center();
    sensors.set_steering_mode(STEERING_OFF);
    motion.set_target_velocity(0);
    turn_IP90L();
    float distance = SENSING_POSITION - HALF_CELL;
    sensors.set_steering_mode(STEER_NORMAL);
    motion.move(distance, speed_parameters->forward_speed, speed_parameters->forward_speed, speed_parameters->acceleration);
    motion.set_position(SENSING_POSITION);
    //turn_smooth(SS90EL);
    m_heading = left_from(m_heading);
  }

//***************************************************************************//
  void turn_right_ip() {
    stop_at_center();
    sensors.set_steering_mode(STEERING_OFF);
    motion.set_target_velocity(0);
    turn_IP90R();
    float distance = SENSING_POSITION - HALF_CELL;
    sensors.set_steering_mode(STEER_NORMAL);
    motion.move(distance, speed_parameters->forward_speed, speed_parameters->forward_speed, speed_parameters->acceleration);
    motion.set_position(SENSING_POSITION);
    //turn_smooth(SS90ER);
    m_heading = right_from(m_heading);
  }

  //***************************************************************************//
  /***
   * As with all the search turns, this command will be called after the robot has
   * reached the search decision point and decided its next move. It is not known
   * how long that takes or what the exact position will be.
   *
   * Turning around is always going to be an in-place operation so it is important
   * that the robot is stationary and as well centred as possible.
   *
   * It only takes 27mm of travel to come to a halt from normal search speed.
   */
  void turn_back() {
    reporter.log_action_status('B', ' ', m_location, m_heading);
    stop_at_center();
    sensors.set_steering_mode(STEERING_OFF);
    // Turn away from the nearest wall
    if(sensors.rss.value >= sensors.lss.value)
    {
      turn_IP90L();
      // Back into wall and re-centre
      motion.move(-(BACK_WALL_TO_CENTER+10), speed_parameters->backing_speed, 0, speed_parameters->acceleration);
      motors.reset_controllers();
      motion.move(BACK_WALL_TO_CENTER, speed_parameters->backing_speed, 0, speed_parameters->acceleration);
      turn_IP90L();
    }
    else
    {
      turn_IP90R();
      // Back into wall and re-centre
      motion.move(-(BACK_WALL_TO_CENTER+10), speed_parameters->backing_speed, 0, speed_parameters->acceleration);
      motors.reset_controllers();
      motion.move(BACK_WALL_TO_CENTER, speed_parameters->backing_speed, 0, speed_parameters->acceleration);
      turn_IP90R();
    }
    // Back into wall
    motion.move(-(BACK_WALL_TO_CENTER+10), speed_parameters->backing_speed, 0, speed_parameters->acceleration);
    motors.reset_controllers();
    sensors.set_steering_mode(STEER_NORMAL);
    float distance = SENSING_POSITION - (HALF_CELL -  BACK_WALL_TO_CENTER);
    motion.move(distance, speed_parameters->forward_speed, speed_parameters->forward_speed, speed_parameters->acceleration);
    motion.set_position(SENSING_POSITION);
    m_heading = behind_from(m_heading);
  }

  //***************************************************************************//
  /***
   * A simple wall follower that knows where it is
   * It will follow the left wall until it reaches the supplied taget
   * cell.
   */
  void follow_to(Location target) {
    SerialPort.println(F("Follow TO"));
    m_handStart = true;
    m_location = START;
    m_heading = NORTH;
    maze.initialise();
    sensors.wait_for_user_start();
    sensors.enable();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    if (not m_handStart) {
      // back up to the wall behind
      // TODO: what if there is not a wall?
      // perhaps the caller should decide so this ALWAYS starts at the cell centre?
      motion.move(-BACK_WALL_TO_CENTER, speed_parameters->backing_speed, 0, speed_parameters->acceleration);
    }
    motion.move(BACK_WALL_TO_CENTER, speed_parameters->forward_speed, speed_parameters->forward_speed, speed_parameters->acceleration);
    motion.set_position(HALF_CELL);
    SerialPort.println(F("Off we go..."));
    motion.wait_until_position(SENSING_POSITION);
    // at the start of this loop we are always at the sensing point
    while (m_location != target) {
      if (switches.button_pressed()) {
        break;
      }
      SerialPort.println();
      reporter.log_action_status('-', ' ', m_location, m_heading);
      sensors.set_steering_mode(STEER_NORMAL);
      m_location = m_location.neighbour(m_heading);
      update_map();
      SerialPort.write(' ');
      SerialPort.write('|');
      SerialPort.write(' ');
      char action = '#';
      if (m_location != target) {
        if (!sensors.see_left_wall) {
          indicators.indicateLeftTurn();
          turn_left();
          action = 'L';
        } else if (!sensors.see_front_wall) {
          indicators.indicateForward();
          move_ahead();
          action = 'F';
        } else if (!sensors.see_right_wall) {
          indicators.indicateRightTurn();
          turn_right();
          action = 'R';
        } else {
          indicators.indicateAboutTurn();
          turn_back();
          action = 'B';
        }
      }
      reporter.log_action_status(action, ' ', m_location, m_heading);
    }
    // we are entering the target cell so come to an orderly
    // halt in the middle of that cell
    stop_at_center();
    SerialPort.println();
    SerialPort.println(F("Arrived!  "));
    delay(250);
    sensors.disable();
    motion.reset_drive_system();
    motion.disable_drive();
    sensors.set_steering_mode(STEERING_OFF);
    indicators.indicateTurnsOff();
    indicators.blink(4, 0, 16, 0); // Green
  }

  /***
    Seach the maze by wall-following and back again
  ****/
  void wall_follow()
  {
      mouse.m_handStart = true;
      mouse.follow_to(maze.goal());
      m_handStart = false;  // Assumes central in a cell
      mouse.follow_to(START);
  }
  
  /***
   * This function moves the mouse from the start position to a target location
   * which is specified as a distance from the start position.
   *
   * Use this function to help with calibration of the encoders and wheel
   * diameter.
   *
   * You could also modify it to generate sensor data or control
   * teemetry while running.
   *
   * @param mm the distance from the start location to the target position
   */
  void run(int mm) {
    Serial.println(F("Follow TO"));
    m_handStart = true;
    m_location = START;
    m_heading = NORTH;
    maze.initialise();
    sensors.wait_for_user_start();
    sensors.enable();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEER_NORMAL);
    motion.move(BACK_WALL_TO_CENTER, speed_parameters->forward_speed, speed_parameters->forward_speed, speed_parameters->acceleration);
    motion.set_position(HALF_CELL);
    motion.move(mm, speed_parameters->forward_speed, 0,  speed_parameters->acceleration);
    Serial.println();
    Serial.println(F("Arrived!  "));
    delay(250);
    sensors.disable();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
  }

  /****************************************************************************/
  /***
   * search_to will cause the mouse to move to the given target cell
   * using safe, exploration speeds and turns.
   *
   * During the search, walls will be mapped but only when first seen.
   * A wall will not be changed once it has been mapped.
   *
   * It is possible for the mapping process to make the mouse think it
   * is walled in with no route to the target if wals are falsely
   * identified as present.
   *
   * On entry, the mouse will know its location and heading and
   * will begin by moving forward. The assumption is that the mouse
   * is already facing in an appropriate direction.
   *
   * All paths will start with a straight.
   *
   * If the function is called with handstart set true, you can
   * assume that the mouse is already backed up to the wall behind.
   *
   * Otherwise, the mouse is assumed to be centrally placed in a cell
   * and may be stationary or moving.
   *
   * The walls for the current location are assumed to be correct in
   * the map since mapping is always done by looking ahead into the
   * cell that is about to be entered.
   *
   * On exit, the mouse will be centered in the target cell still
   * facing in the direction it entered that cell. This will
   * always be one of the four cardinal directions NESW
   *
   */

  void search_to(Location target) {
    maze.set_mask(MASK_OPEN);   // Treat unknown cell walls as open gaps
    maze.flood(target);
    delay(200);
    sensors.enable();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);  // never steer from zero speed
    if (not m_handStart) {
      // back up to the wall behind
      // TODO: what if there is not a wall?
      // perhaps the caller should decide so this ALWAYS starts at the cell centre?
      motion.move(-BACK_WALL_TO_CENTER, speed_parameters->backing_speed, 0, speed_parameters->acceleration);
    }
    motion.move(BACK_WALL_TO_CENTER, speed_parameters->forward_speed, speed_parameters->forward_speed, speed_parameters->acceleration);
    motion.set_position(HALF_CELL);
    SerialPort.print(F("Off we go..."));
    SerialPort.print('[');
    SerialPort.print(target.x);
    SerialPort.print(',');
    SerialPort.print(target.y);
    SerialPort.print(']');
    SerialPort.println();

    motion.wait_until_position(SENSING_POSITION);
    // Each iteration of this loop starts at the sensing point
    while (m_location != target) {
      if (switches.button_pressed()) {  // allow user to abort gracefully
        break;
      }
      SerialPort.println();
      //SerialPort.println(encoders.robot_distance());
#ifdef DEBUG_LOGGING
      reporter.report_profile();
      reporter.print_wall_sensors();
#endif
      reporter.log_action_status('-', ' ', m_location, m_heading);
      sensors.set_steering_mode(STEER_NORMAL);
      m_location = m_location.neighbour(m_heading);  // the cell we are about to enter
      update_map();
      maze.flood(target);
      unsigned char newHeading = maze.heading_to_smallest(m_location, m_heading);
      unsigned char hdgChange = (newHeading - m_heading) & 0x3;
      if (m_location != target) {
        if(newHeading == BLOCKED )//|| m_location.y > 1)
        {
          motion.stop_move();
          for(int i = 0; i < 5; i++)
          {
#ifdef DEBUG_LOGGING
            //SerialPort.println(encoders.robot_distance());
            SerialPort.println();
            reporter.report_profile();
            reporter.print_wall_sensors();
            reporter.log_action_status('P', 'P', m_location, m_heading);
#endif
            delay(10);
          }
          motion.stop();
          sensors.set_steering_mode(STEERING_OFF);
          sensors.disable();
          motion.disable_drive();
          SerialPort.println();
          SerialPort.println(F("NO ROUTE TO TARGET!"));
          panic();
          return;
        }
        switch (hdgChange) {
          // each of the following actions will finish with the
          // robot moving and at the sensing point ready for the
          // next loop iteration
          case AHEAD:
            indicators.indicateForward();
            move_ahead();
            break;
          case RIGHT:
            indicators.indicateRightTurn();
            turn_right();
            break;
          case BACK:
            indicators.indicateAboutTurn();
            turn_back();
            break;
          case LEFT:
            indicators.indicateLeftTurn();
            turn_left();
            break;
        }
      }
    }
    // we are entering the target cell so come to an orderly
    // halt in the middle of that cell
    stop_at_center();
    sensors.disable();
    SerialPort.println();
    SerialPort.println(F("Arrived!  "));
    delay(250);
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    indicators.indicateTurnsOff();
    indicators.blink(4, 0, 16, 0); // Green
}

  /****************************************************************************/
  bool getRandomBool() {
    return rand() % 2 == 0;
  }

  /****************************************************************************/
  // when searching the maze select a random direction for the next cell
  uint8_t randomHeading() {
    uint8_t turnDirection;
    bool leftWall = sensors.see_left_wall;
    bool rightWall = sensors.see_right_wall;
    bool frontWall = sensors.see_front_wall;

    if (leftWall && rightWall && frontWall) {
      turnDirection = BACK;
    } else if (leftWall && rightWall) {
      turnDirection = AHEAD;
    } else if (rightWall && frontWall) {
      turnDirection = LEFT;
    } else if (leftWall && frontWall) {
      turnDirection = RIGHT;
    } else if (leftWall) {
      if (getRandomBool()) {
        turnDirection = RIGHT;
      } else {
        turnDirection = AHEAD;
      }
    } else if (rightWall) {
      if (getRandomBool()) {
        turnDirection = LEFT;
      } else {
        turnDirection = AHEAD;
      }
    } else {
      if (getRandomBool()) {
        turnDirection = LEFT;
      } else {
        turnDirection = RIGHT;
      }
    }

    return turnDirection;
  }

  /****************************************************************************/
  void wander_to(Location target) {
    target = Location(16, 16);
    Serial.println(F("Wandering..."));
    m_handStart = true;
    m_location = START;
    m_heading = NORTH;
    maze.initialise();
    sensors.wait_for_user_start();
    sensors.enable();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    motion.move(BACK_WALL_TO_CENTER, speed_parameters->forward_speed, speed_parameters->forward_speed, speed_parameters->acceleration);
    motion.set_position(HALF_CELL);
    Serial.println(F("Search. Off we go..."));
    motion.wait_until_position(SENSING_POSITION);
    // at the start of this loop we are always at the sensing point
    while (m_location != target) {
      if (switches.button_pressed()) {
        break;
      }
      Serial.println();
      reporter.log_action_status('-', ' ', m_location, m_heading);
      sensors.set_steering_mode(STEER_NORMAL);
      m_location = m_location.neighbour(m_heading);
      update_map();
      Serial.write(' ');
      Serial.write('|');
      Serial.write(' ');
      char action = 'W';
      uint8_t hdg = randomHeading();
      if (hdg == LEFT) {
        turn_left();
        action = 'L';
      } else if (hdg == AHEAD) {
        move_ahead();
        action = 'F';
      } else if (hdg == RIGHT) {
        turn_right();
        action = 'R';
      } else {
        turn_back();
        action = 'B';
      }
      reporter.log_action_status(action, ' ', m_location, m_heading);
    }
    // we are entering the target cell so come to an orderly
    // halt in the middle of that cell
    stop_at_center();
    Serial.println();
    Serial.println(F("Arrived!  "));
    delay(250);
    sensors.disable();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
  }

  /****************************************************************************/
  /***
   * run_to should take the mouse to the target cell by whatever
   * fast means it has. There is no mapping done, just the motion.
   *
   * It should be assumed that the maze is flooded using the CLOSED mask
   * so that the route is safe.
   *
   * run_to must calculate the path itself. This may be either by
   * pre-calculation to generate a series of operations or the path
   * may be calculated on-the-fly using the cost map from the flood.
   *
   * On entry, the mouse will know its location and heading so the
   * first operation will be to turn to face the right way for
   * the initial move. All paths will start with a straight.
   *
   * If the function is called with handstart set true, you can
   * assume that the mouse is already backed up to the wall behind.
   *
   * On exit, the mouse will be centered in the target cell still
   * facing in the direction it entered that cell. This will
   * always be one of the four cardinal directions NESW
   */
  void run_to(Location target) {
    maze.set_mask(MASK_CLOSED);   // Treat unknown cell walls as blocked, so we don't enter theses cells
    maze.flood(target);
    delay(200);
    sensors.enable();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);  // never steer from zero speed
    if (not m_handStart) {
      // back up to the wall behind
      // TODO: what if there is not a wall?
      // perhaps the caller should decide so this ALWAYS starts at the cell centre?
      motion.move(-BACK_WALL_TO_CENTER, speed_parameters->backing_speed, 0, speed_parameters->acceleration);
    }
    motion.move(BACK_WALL_TO_CENTER, speed_parameters->forward_speed, speed_parameters->forward_speed, speed_parameters->acceleration);
    motion.set_position(HALF_CELL);
    SerialPort.print(F("Speed run. Off we go..."));
    SerialPort.print('[');
    SerialPort.print(target.x);
    SerialPort.print(',');
    SerialPort.print(target.y);
    SerialPort.print(']');
    SerialPort.println();

    motion.wait_until_position(SENSING_POSITION);
    // Each iteration of this loop starts at the sensing point
    while (m_location != target) {
      if (switches.button_pressed()) {  // allow user to abort gracefully
        break;
      }
      SerialPort.println();
#ifdef DEBUG_LOGGING
      reporter.print_wall_sensors();
#endif
      reporter.log_action_status('-', ' ', m_location, m_heading);
      sensors.set_steering_mode(STEER_NORMAL);
      m_location = m_location.neighbour(m_heading);  // the cell we are about to enter
      //update_map();
      //maze.flood(target);
      unsigned char newHeading = maze.heading_to_smallest(m_location, m_heading);
      unsigned char hdgChange = (newHeading - m_heading) & 0x3;
      if (m_location != target) {
        if(newHeading == BLOCKED)
        {
          motion.stop();
          sensors.set_steering_mode(STEERING_OFF);
          sensors.disable();
          motion.disable_drive();
          SerialPort.println();
          SerialPort.println(F("NO ROUTE TO TARGET!"));
          panic();
          return;
        }
        switch (hdgChange) {
          // each of the following actions will finish with the
          // robot moving and at the sensing point ready for the
          // next loop iteration
          case AHEAD:
            indicators.indicateForward();
            move_ahead();
            break;
          case RIGHT:
            indicators.indicateRightTurn();
            turn_right();
            break;
          case BACK:
            indicators.indicateAboutTurn();
            turn_back();
            break;
          case LEFT:
            indicators.indicateLeftTurn();
            turn_left();
            break;
        }
      }
    }
    // we are entering the target cell so come to an orderly
    // halt in the middle of that cell
    stop_at_center();
    sensors.disable();
    SerialPort.println();
    SerialPort.println(F("Arrived!  "));
    delay(250);
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    indicators.indicateTurnsOff();
    indicators.blink(4, 0, 16, 0); // Green
  }

  /****************************************************************************/
  void turn_to_face(Heading newHeading) {
    unsigned char hdgChange = (newHeading + HEADING_COUNT - m_heading) % HEADING_COUNT;
    switch (hdgChange) {
      case AHEAD:
        break;
      case RIGHT:
        turn_IP90R();
        break;
      case BACK:
        turn_IP180();
        break;
      case LEFT:
        turn_IP90L();
        break;
    }
    m_heading = newHeading;
  }

  /****************************************************************************/
  void update_map() {
    bool leftWall = sensors.see_left_wall;
    bool frontWall = sensors.see_front_wall;
    bool rightWall = sensors.see_right_wall;
    char w[] = "--- ";
    if (leftWall) {
      w[0] = 'L';
    };
    if (frontWall) {
      w[1] = 'F';
    };
    if (rightWall) {
      w[2] = 'R';
    };
    SerialPort.print(w);
    switch (m_heading) {
      case NORTH:
        maze.update_wall_state(m_location, NORTH, frontWall ? WALL : EXIT);
        maze.update_wall_state(m_location, EAST, rightWall ? WALL : EXIT);
        maze.update_wall_state(m_location, WEST, leftWall ? WALL : EXIT);
        break;
      case EAST:
        maze.update_wall_state(m_location, EAST, frontWall ? WALL : EXIT);
        maze.update_wall_state(m_location, SOUTH, rightWall ? WALL : EXIT);
        maze.update_wall_state(m_location, NORTH, leftWall ? WALL : EXIT);
        break;
      case SOUTH:
        maze.update_wall_state(m_location, SOUTH, frontWall ? WALL : EXIT);
        maze.update_wall_state(m_location, WEST, rightWall ? WALL : EXIT);
        maze.update_wall_state(m_location, EAST, leftWall ? WALL : EXIT);
        break;
      case WEST:
        maze.update_wall_state(m_location, WEST, frontWall ? WALL : EXIT);
        maze.update_wall_state(m_location, NORTH, rightWall ? WALL : EXIT);
        maze.update_wall_state(m_location, SOUTH, leftWall ? WALL : EXIT);
        break;
      default:
        // This is an error. We should handle it.
        break;
    }
  }

  /****************************************************************************/
  /***
   * The mouse is expected to be in the start cell heading NORTH
   * The maze may, or may not, have been searched.
   * There may, or may not, be a solution.
   *
   * This simple searcher will just search to goal, turn around and
   * search back to the start. At that point there will be a route
   * but it is unlikely to be optimal.
   *
   * the mouse can run this route by creating a path that does not
   * pass through unvisited cells.
   *
   * A better searcher will continue until a path generated through all
   * cells, regardless of visited state, does not pass through any
   * unvisited cells.
   *
   * The return value is not currently used but could indicate whether
   * the maze is 'solved'. That is, whether there is any need to search
   * further.
   *
   */
  int search_maze() {
    sensors.wait_for_user_start();
    SerialPort.println(F("Search TO"));
    m_handStart = true; // Assumes backed-up against a wall
    m_location = START;
    m_heading = NORTH;
    search_to(maze.goal());

    // Search back to start
    maze.flood(START);
    Heading best_direction = maze.heading_to_smallest(m_location, m_heading);
    turn_to_face(best_direction);
    m_handStart = false;  // Assumes central in a cell
    search_to(START);

    // Rotate and back up to original start position
    turn_to_face(NORTH);
    motion.move(-(BACK_WALL_TO_CENTER+10), speed_parameters->backing_speed, 0, speed_parameters->acceleration);
    motion.stop();
    motion.disable_drive();
    return 0;
  }

/****************************************************************************/
  /***
   * The mouse is expected to be in the start cell heading NORTH
   * The maze may, or may not, have been searched.
   * There may, or may not, be a solution.
   *
   * This simple searcher will just search to goal, turn around and
   * search back to the start. At that point there will be a route
   * but it is unlikely to be optimal.
   *
   * the mouse can run this route by creating a path that does not
   * pass through unvisited cells.
   *
   * A better searcher will continue until a path generated through all
   * cells, regardless of visited state, does not pass through any
   * unvisited cells.
   *
   * The return value is not currently used but could indicate whether
   * the maze is 'solved'. That is, whether there is any need to search
   * further.
   *
   */
  int fast_run() {
    sensors.wait_for_user_start();
    SerialPort.println(F("Fast run TO"));
    m_handStart = true; // Assumes backed-up against a wall
    m_location = START;
    m_heading = NORTH;
    run_to(maze.goal());

    // Search back to start, in case this yields anything better
    speed_parameters = &speed_parameters_base;  
    turn_params = turn_params_base;
    maze.flood(START);
    Heading best_direction = maze.heading_to_smallest(m_location, m_heading);
    turn_to_face(best_direction);
    m_handStart = false;  // Assumes central in a cell
    search_to(START);

    // Rotate and back up to original start position
    turn_to_face(NORTH);
    motion.move(-(BACK_WALL_TO_CENTER+10), speed_parameters->backing_speed, 0, speed_parameters->acceleration);
    motion.stop();
    motion.disable_drive();
    return 0;
  }

  //***************************************************************************//
  //************  BELOW HERE ARE VARIOUS TEST FUNCTIONS ***********************//
  //********** THEY ARE NOT ESSENTIAL TO THE BUSINESS OF **********************//
  //******** SOLVING THE MAZE BUT THEY MAY HELP WITH SETUP ********************//
  //***************************************************************************//

  /***
   * Visual feedback by flashing the LED indicators
   */
  void blink(int count) {
    indicators.blink(count, 8, 0, 0); // Red
  }

  /***
   * just sit in a loop, flashing lights waiting for the button to be pressed
   */
  void panic() {
    while (!switches.button_pressed()) {
      blink(1);
    }
    switches.wait_for_button_release();
    indicators.showReadyIndicator(0);
    //digitalWrite(LED_BUILTIN, 0);
  }

  /***
   * You may want to log the front sensor readings as a function of distance
   * from the wall. This function does that. Place the robot hard up against
   * a wall ahead and run the command. You will get a table of values for
   * the sensors as a function of distance.
   *
   */
  void conf_log_front_sensor() {
    sensors.enable();
    motion.reset_drive_system();
    reporter.front_sensor_track_header();
    motion.start_move(-200, 100, 0, 500);
    while (not motion.move_finished()) {
      reporter.front_sensor_track();
    }
    motion.reset_drive_system();
    motion.disable_drive();
    sensors.set_steering_mode(STEERING_OFF);
    sensors.disable();
  }

/* Simple test of encoder info
*/
void test_log_position_sensors() {
    sensors.enable();
    motion.reset_drive_system();
    reporter.report_profile_header();
    while (not switches.button_pressed()) {
      reporter.report_profile();
      delay(50);
    }
    motion.reset_drive_system();
    sensors.disable();
  }

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

  void conf_sensor_spin_calibrate() {
    int side = sensors.wait_for_user_start();  // cover front sensor with hand to start
    bool use_raw = (side == LEFT_START) ? true : false;
    sensors.enable();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    reporter.report_sensor_track_header();
    motion.start_turn(360, 180, 0, 1800);
    while (not motion.turn_finished()) {
      reporter.report_radial_track(use_raw);
      // reporter.print_wall_sensors();
    }
    reporter.report_radial_track(use_raw);
    motion.reset_drive_system();
    motion.disable_drive();
    delay(100);
  }

  /**
   * TODO: move this out to the mazerunner-setup code
   *
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

  void conf_edge_detection() {
    bool left_edge_found = false;
    bool right_edge_found = false;
    int left_edge_position = 0;
    int right_edge_position = 0;
    int left_max = 0;
    int right_max = 0;
    sensors.wait_for_user_start();  // cover front sensor with hand to start
    sensors.enable();
    delay(100);
    motion.reset_drive_system();
    sensors.set_steering_mode(STEER_NORMAL);
    SerialPort.println(F("Edge positions:"));
    motion.start_move(FULL_CELL * 4, 500, 0, 1000);
    while (not motion.move_finished()) {
      if (sensors.lss.value > left_max) {
        left_max = sensors.lss.value;
      }

      if (sensors.rss.value > right_max) {
        right_max = sensors.rss.value;
      }

      if (not left_edge_found) {
        if (sensors.lss.value < left_max / 2) {
          left_edge_position = int(0.5 + motion.position());
          left_edge_found = true;
        }
      }
      if (not right_edge_found) {
        if (sensors.rss.value < right_max / 2) {
          right_edge_position = int(0.5 + motion.position());
          right_edge_found = true;
        }
      }
      delay(5);
    }
    SerialPort.println(encoders.robot_distance());
    SerialPort.print(F("Left: "));
    if (left_edge_found) {
      SerialPort.print(BACK_WALL_TO_CENTER + left_edge_position);
    } else {
      SerialPort.print('-');
    }

    SerialPort.print(F("  Right: "));
    if (right_edge_found) {
      SerialPort.print(BACK_WALL_TO_CENTER + right_edge_position);
    } else {
      SerialPort.print('-');
    }
    SerialPort.println();
    motion.reset_drive_system();
    motion.disable_drive();
    sensors.set_steering_mode(STEERING_OFF);
    sensors.disable();
    delay(100);
  }

  /***
   * A basic function to let you test the configuration of the SS90Ex turns.
   *
   * These are the turns used during the search of the maze and need to be
   * accurate and repeatable.
   *
   * You may need to spend some time with this function to get the turns
   * just right.
   *
   * NOTE: that the turn parameters are stored in the robot config file
   * NOTE: that the left and right turns are likely to be different.
   *
   */
  void test_SS90E() {
    // note that changes to the speeds are likely to affect
    // the other turn parameters
    uint8_t side = sensors.wait_for_user_start();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    // move to the boundary with the next cell
    float distance = BACK_WALL_TO_CENTER + HALF_CELL + SENSING_POSITION;  // From backed-up to wall, travel to sensing position in 2nd cell
    motion.move(distance,  speed_parameters->forward_speed, speed_parameters->forward_speed,  speed_parameters->acceleration);
    motion.set_position(SENSING_POSITION);

    if (side == RIGHT_START) {
      turn_smooth(SS90ER);
    } else {
      turn_smooth(SS90EL);
    }
    // after the turn, estimate the angle error by looking for
    // changes in the side sensor readings
    int sensor_left = sensors.lss.value;
    int sensor_right = sensors.rss.value;
    // move a full cell. The resting position of the mouse have the
    // same offset as the turn ending
    motion.move(FULL_CELL, speed_parameters->forward_speed, 0,  speed_parameters->acceleration);
    sensor_left -= sensors.lss.value;
    sensor_right -= sensors.rss.value;
    reporter.print_justified(sensor_left, 5);
    reporter.print_justified(sensor_right, 5);
    motion.reset_drive_system();
    motion.disable_drive();
    sensors.set_steering_mode(STEERING_OFF);
  }

void test_SS90E_Right() {
    // note that changes to the speeds are likely to affect
    // the other turn parameters
    uint8_t side = sensors.wait_for_user_start();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    // move to the boundary with the next cell
    float distance = BACK_WALL_TO_CENTER + HALF_CELL;
    motion.move(distance, speed_parameters->forward_speed, speed_parameters->forward_speed,  speed_parameters->acceleration);
    motion.set_position(FULL_CELL);

//    if (side == RIGHT_START) {
      turn_smooth(SS90ER);
//    } else {
//      turn_smooth(SS90EL);
//    }
    // after the turn, estimate the angle error by looking for
    // changes in the side sensor readings
    int sensor_left = sensors.lss.value;
    int sensor_right = sensors.rss.value;
    // move two cells. The resting position of the mouse have the
    // same offset as the turn ending
    motion.move(2 * FULL_CELL, speed_parameters->forward_speed, 0,  speed_parameters->acceleration);
    sensor_left -= sensors.lss.value;
    sensor_right -= sensors.rss.value;
    reporter.print_justified(sensor_left, 5);
    reporter.print_justified(sensor_right, 5);
    motion.reset_drive_system();
    motion.disable_drive();
    sensors.set_steering_mode(STEERING_OFF);
  }

/*
  Test turns by making a zig-zag move: 
    Ahead
    Right
    Ahead
    Left
    Ahead
    Stop
*/
  void test_turn_right_left() {
    // note that changes to the speeds are likely to affect
    // the other turn parameters
    uint8_t side = sensors.wait_for_user_start();
    sensors.enable();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    // move to the boundary with the next cell
    float distance = BACK_WALL_TO_CENTER;

    SerialPort.println("Accelerating");

    motion.move(distance, speed_parameters->forward_speed, speed_parameters->forward_speed,  speed_parameters->acceleration);
    motion.set_position(HALF_CELL);

    SerialPort.println("Turning right");
    turn_right();
    SerialPort.println("Ahead");
    move_ahead();
    SerialPort.println("Turning left");
    turn_left();
    SerialPort.println("Ahead");
    move_ahead();
    SerialPort.println("Stopping");

    motion.reset_drive_system();
    motion.disable_drive();
    sensors.set_steering_mode(STEERING_OFF);
    sensors.disable();
  }

  /***
   * A basic function to let you test the configuration of forward moves.
   *
   * These are the turns used during the search of the maze and need to be
   * accurate and repeatable.
   *
   * You may need to spend some time with this function to get the move
   * to drive straight
   *
   * NOTE: that the motor scale parameters are stored in the robot config file
   * NOTE: that the left and right motor powers are likely to be different.
   *
   */
  void test_forward(int distance) {
    // note that changes to the speeds are likely to affect
    // the other turn parameters
    sensors.wait_for_user_start();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    reporter.print_justified(encoders.robot_distance(), 5);
    // move 
    motion.move(distance, speed_parameters->forward_speed, 0,  speed_parameters->acceleration);
    //motion.turn(90, speed_parameters->forward_speed, 0,  speed_parameters->acceleration);
    // Be sure robot has come to a halt.
    motion.stop();
    // Show actual distance measured
    reporter.print_justified(encoders.robot_distance(), 5);
    // return to idle
    motion.reset_drive_system();
    motion.disable_drive();
    sensors.set_steering_mode(STEERING_OFF);
  }

  /***
   * loop until the user button is pressed while
   * pumping out sensor readings. The first four numbers are
   * the raw readings, the next four are normalised then there
   * are two values for the sum and difference of the front sensors
   *
   * The advanced user might use this as a start for auto calibration
   */
  void show_sensor_calibration() {
    reporter.wall_sensor_header();
    sensors.enable();
    while (not switches.button_pressed()) {
      reporter.print_wall_sensors();
    }
    switches.wait_for_button_release();
    SerialPort.println();
    delay(200);
    sensors.disable();
  }

  void test_raw_sensors()
  {
    sensors.enable();
    while (not switches.button_pressed()) {
      reporter.print_raw_sensors();
      delay(20);
    }
    switches.wait_for_button_release();
    SerialPort.println();
    delay(200);
    sensors.disable();
  }

 private:
  Heading m_heading;
  Location m_location;
  bool m_handStart = false;
};

#endif  // MOUSE_H