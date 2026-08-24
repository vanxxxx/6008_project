/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description: Simplistic drone control:
 * - Stabilize the robot using the embedded sensors.
 * - Use PID technique to stabilize the drone roll/pitch/yaw.
 * - Use position and vertical-speed feedback to stabilize altitude.
 * - Stabilize the camera.
 * - Control the robot using the computer keyboard.
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>

#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>

#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

static double wrap_angle(double angle) {
  const double pi = 3.14159265358979323846;
  while (angle > pi)
    angle -= 2.0 * pi;
  while (angle < -pi)
    angle += 2.0 * pi;
  return angle;
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  wb_robot_init();
  const int timestep = (int)wb_robot_get_basic_time_step();
  const double dt = timestep / 1000.0;

  // Get and enable devices.
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag front_left_led = wb_robot_get_device("front left led");
  WbDeviceTag front_right_led = wb_robot_get_device("front right led");
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  wb_keyboard_enable(timestep);
  WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
  WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");
  const double camera_roll_max_velocity = wb_motor_get_max_velocity(camera_roll_motor);
  const double camera_pitch_max_velocity = wb_motor_get_max_velocity(camera_pitch_motor);
  const double camera_roll_min_position = wb_motor_get_min_position(camera_roll_motor);
  const double camera_roll_max_position = wb_motor_get_max_position(camera_roll_motor);
  const double camera_pitch_min_position = wb_motor_get_min_position(camera_pitch_motor);
  const double camera_pitch_max_position = wb_motor_get_max_position(camera_pitch_motor);

  // Get propeller motors and set them to velocity mode.
  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
  for (int m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }

  printf("Start the drone...\n");

  // Wait one second for the sensors to provide valid values.
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0)
      break;
  }

  printf("You can control the drone with your computer keyboard:\n");
  printf("- 'up': move forward.\n");
  printf("- 'down': move backward.\n");
  printf("- 'right': turn right.\n");
  printf("- 'left': turn left.\n");
  printf("- 'shift + up': increase the target altitude.\n");
  printf("- 'shift + down': decrease the target altitude.\n");
  printf("- 'shift + right': strafe right.\n");
  printf("- 'shift + left': strafe left.\n");
  printf("- hold 'Q': tilt left by 2 degrees.\n");
  printf("- hold 'E': tilt right by 2 degrees.\n");
  printf("- press 'P': stop/resume all motors and LEDs.\n");

  // Constants, empirically found.
  const double k_vertical_thrust = 68.5;
  const double k_vertical_offset = 0.6;
  const double k_vertical_p = 3.0;
  const double k_vertical_d = 3.0;  // Damping that suppresses repeated altitude overshoot.
  const double k_roll_p = 50.0;
  const double k_pitch_p = 30.0;
  const double k_yaw_p = 2.0;
  const double k_yaw_d = 0.4;
  const double k_qe_tilt_angle = 5.0 * 3.14159265358979323846 / 180.0;
  const double k_move_tilt_angle = 2.5 * 3.14159265358979323846 / 180.0;
  const double k_tilt_slew_rate = 22.0 * 3.14159265358979323846 / 180.0;
  const double k_position_p = 0.08;  // Return decisively to the point where the movement key was released.
  const double k_position_d = 0.18;  // Stronger horizontal braking for a distinct stop/reverse response.
  const double k_max_hold_tilt = 2.0 * 3.14159265358979323846 / 180.0;
  const double vertical_speed_filter_time = 0.15;
  const double horizontal_speed_filter_time = 0.20;

  double target_altitude = 1.0;
  double previous_altitude = 0.0;
  double filtered_vertical_speed = 0.0;
  bool altitude_initialized = false;
  double target_x = 0.0;
  double target_y = 0.0;
  double previous_x = 0.0;
  double previous_y = 0.0;
  double filtered_velocity_x = 0.0;
  double filtered_velocity_y = 0.0;
  bool horizontal_position_initialized = false;
  double target_yaw = 0.0;
  bool heading_initialized = false;
  double commanded_roll = 0.0;
  double commanded_pitch = 0.0;
  bool stopped = false;
  bool p_was_pressed = false;

  while (wb_robot_step(timestep) != -1) {
    const double time = wb_robot_get_time();

    // Retrieve robot position and angular velocity using the sensors.
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    const double roll = rpy[0];
    const double pitch = rpy[1];
    const double yaw = rpy[2];
    const double *position = wb_gps_get_values(gps);
    const double position_x = position[0];
    const double position_y = position[1];
    const double altitude = position[2];
    const double *angular_velocity = wb_gyro_get_values(gyro);
    const double roll_velocity = angular_velocity[0];
    const double pitch_velocity = angular_velocity[1];
    const double yaw_velocity = angular_velocity[2];

    if (!heading_initialized) {
      target_yaw = yaw;
      heading_initialized = true;
    }

    // Estimate vertical speed and filter GPS differentiation noise.
    if (!altitude_initialized) {
      previous_altitude = altitude;
      altitude_initialized = true;
    }
    const double raw_vertical_speed = (altitude - previous_altitude) / dt;
    previous_altitude = altitude;
    const double speed_filter_alpha = dt / (vertical_speed_filter_time + dt);
    filtered_vertical_speed += speed_filter_alpha * (raw_vertical_speed - filtered_vertical_speed);

    // Estimate horizontal velocity and remember the initial hover position.
    if (!horizontal_position_initialized) {
      target_x = position_x;
      target_y = position_y;
      previous_x = position_x;
      previous_y = position_y;
      horizontal_position_initialized = true;
    }
    const double raw_velocity_x = (position_x - previous_x) / dt;
    const double raw_velocity_y = (position_y - previous_y) / dt;
    previous_x = position_x;
    previous_y = position_y;
    const double horizontal_filter_alpha = dt / (horizontal_speed_filter_time + dt);
    filtered_velocity_x += horizontal_filter_alpha * (raw_velocity_x - filtered_velocity_x);
    filtered_velocity_y += horizontal_filter_alpha * (raw_velocity_y - filtered_velocity_y);

    double manual_roll_target = 0.0;
    double manual_pitch_target = 0.0;
    double yaw_disturbance = 0.0;
    bool q_is_pressed = false;
    bool e_is_pressed = false;
    bool p_is_pressed = false;

    // Read every key reported during this simulation step.
    int key = wb_keyboard_get_key();
    while (key > 0) {
      // Keyboard modifiers occupy the upper bits; mask them for letter keys.
      const int base_key = key & 0xFFFF;
      if (base_key == 'Q' || base_key == 'q')
        q_is_pressed = true;
      else if (base_key == 'E' || base_key == 'e')
        e_is_pressed = true;
      else if (base_key == 'P' || base_key == 'p')
        p_is_pressed = true;

      switch (key) {
        case WB_KEYBOARD_UP:
          manual_pitch_target = k_move_tilt_angle;
          break;
        case WB_KEYBOARD_DOWN:
          manual_pitch_target = -k_move_tilt_angle;
          break;
        case WB_KEYBOARD_RIGHT:
          yaw_disturbance = -1.3;
          break;
        case WB_KEYBOARD_LEFT:
          yaw_disturbance = 1.3;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT):
          manual_roll_target = k_move_tilt_angle;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT):
          manual_roll_target = -k_move_tilt_angle;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
          target_altitude += 0.05;
          printf("target altitude: %f [m]\n", target_altitude);
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
          target_altitude -= 0.05;
          printf("target altitude: %f [m]\n", target_altitude);
          break;
      }
      key = wb_keyboard_get_key();
    }

    // Toggle only once per physical P-key press, rather than once per timestep.
    if (p_is_pressed && !p_was_pressed) {
      stopped = !stopped;
      if (stopped) {
        printf("Emergency stop: motors and LEDs are off. Press P to resume.\n");
      } else {
        // Resume from the current pose so the controller does not jump back or create a thrust spike.
        previous_altitude = altitude;
        filtered_vertical_speed = 0.0;
        target_x = position_x;
        target_y = position_y;
        previous_x = position_x;
        previous_y = position_y;
        filtered_velocity_x = 0.0;
        filtered_velocity_y = 0.0;
        target_yaw = yaw;
        commanded_roll = roll;
        commanded_pitch = pitch;
        wb_motor_set_velocity(camera_roll_motor, camera_roll_max_velocity);
        wb_motor_set_velocity(camera_pitch_motor, camera_pitch_max_velocity);
        printf("Flight control resumed.\n");
      }
    }
    p_was_pressed = p_is_pressed;

    if (stopped) {
      // Keep the restart hover point synchronized with the stopped drone's current position.
      target_x = position_x;
      target_y = position_y;
      previous_x = position_x;
      previous_y = position_y;
      filtered_velocity_x = 0.0;
      filtered_velocity_y = 0.0;
      target_yaw = yaw;
      commanded_roll = 0.0;
      commanded_pitch = 0.0;
      wb_led_set(front_left_led, 0);
      wb_led_set(front_right_led, 0);
      for (int m = 0; m < 4; ++m)
        wb_motor_set_velocity(motors[m], 0.0);
      wb_motor_set_velocity(camera_roll_motor, 0.0);
      wb_motor_set_velocity(camera_pitch_motor, 0.0);
      continue;
    }

    // Blink the front LEDs alternately at a one-second rate.
    const bool led_state = ((int)time) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);

    // Stabilize the camera using gyro feedback.
    wb_motor_set_position(
      camera_roll_motor, CLAMP(-0.115 * roll_velocity, camera_roll_min_position, camera_roll_max_position));
    wb_motor_set_position(
      camera_pitch_motor, CLAMP(-0.1 * pitch_velocity, camera_pitch_min_position, camera_pitch_max_position));

    // While translating manually, move the hover point along with the drone. On release,
    // position and velocity feedback brake the remaining motion and hold the new position.
    const bool manual_translation =
      q_is_pressed || e_is_pressed || manual_roll_target != 0.0 || manual_pitch_target != 0.0;
    if (manual_translation) {
      target_x = position_x;
      target_y = position_y;
    }

    double target_roll = manual_roll_target;
    double target_pitch = manual_pitch_target;
    if (!manual_translation) {
      const double position_command_x =
        k_position_p * (target_x - position_x) - k_position_d * filtered_velocity_x;
      const double position_command_y =
        k_position_p * (target_y - position_y) - k_position_d * filtered_velocity_y;

      // Convert world-coordinate GPS corrections into the drone's forward/left axes.
      const double cos_yaw = cos(yaw);
      const double sin_yaw = sin(yaw);
      const double forward_command = cos_yaw * position_command_x + sin_yaw * position_command_y;
      const double left_command = -sin_yaw * position_command_x + cos_yaw * position_command_y;
      target_pitch = CLAMP(forward_command, -k_max_hold_tilt, k_max_hold_tilt);
      target_roll = CLAMP(-left_command, -k_max_hold_tilt, k_max_hold_tilt);
    }

    // Q/E override automatic roll holding while pressed. Pressing both cancels the tilt.
    if (q_is_pressed != e_is_pressed)
      target_roll = q_is_pressed ? -k_qe_tilt_angle : k_qe_tilt_angle;

    // Limit attitude-target slew so a key press cannot create an abrupt motor step.
    const double max_tilt_step = k_tilt_slew_rate * dt;
    commanded_roll += CLAMP(target_roll - commanded_roll, -max_tilt_step, max_tilt_step);
    commanded_pitch += CLAMP(target_pitch - commanded_pitch, -max_tilt_step, max_tilt_step);

    // Compute roll, pitch, yaw and vertical control inputs.
    const double roll_error = CLAMP(roll - commanded_roll, -1.0, 1.0);
    const double roll_input = k_roll_p * roll_error + roll_velocity;
    const double pitch_error = CLAMP(pitch - commanded_pitch, -1.0, 1.0);
    const double pitch_input = k_pitch_p * pitch_error + pitch_velocity;
    double yaw_input = 0.0;
    if (yaw_disturbance != 0.0) {
      // Move the heading-hold target while the user is actively turning.
      target_yaw = yaw;
      yaw_input = yaw_disturbance;
    } else {
      const double yaw_error = wrap_angle(target_yaw - yaw);
      yaw_input = CLAMP(k_yaw_p * yaw_error - k_yaw_d * yaw_velocity, -1.3, 1.3);
    }

    const double altitude_error = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    const double vertical_position_input = k_vertical_p * pow(altitude_error, 3.0);
    const double vertical_input =
      CLAMP(vertical_position_input - k_vertical_d * filtered_vertical_speed, -12.0, 12.0);

    // Mix stabilization commands into the four propeller speeds.
    const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
    wb_motor_set_velocity(front_left_motor, front_left_motor_input);
    wb_motor_set_velocity(front_right_motor, -front_right_motor_input);
    wb_motor_set_velocity(rear_left_motor, -rear_left_motor_input);
    wb_motor_set_velocity(rear_right_motor, rear_right_motor_input);
  }

  wb_robot_cleanup();
  return EXIT_SUCCESS;
}
