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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/robot.h>

#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/plugins/robot_window/robot_wwi.h>

#include "motion_protocol.h"

#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

#define DEFAULT_ACTION_DURATION_MS 500
#define DEFAULT_IDLE_DURATION_MS 500
#define MIN_ACTION_DURATION_MS 100
#define MAX_ACTION_DURATION_MS 5000
#define MIN_IDLE_DURATION_MS 0
#define MAX_IDLE_DURATION_MS 5000

typedef struct {
  MotionTransmission transmission;
  bool has_encoding;
  bool running;
  int action_duration_ms;
  int idle_duration_ms;
  double started_at;
  double last_status_at;
  size_t last_reported_slot;
  bool last_reported_action_phase;
} MotionExecution;

static double wrap_angle(double angle) {
  const double pi = 3.14159265358979323846;
  while (angle > pi)
    angle -= 2.0 * pi;
  while (angle < -pi)
    angle += 2.0 * pi;
  return angle;
}

static int hex_value(char value) {
  if (value >= '0' && value <= '9')
    return value - '0';
  if (value >= 'a' && value <= 'f')
    return value - 'a' + 10;
  if (value >= 'A' && value <= 'F')
    return value - 'A' + 10;
  return -1;
}

static bool decode_hex_message(
  const char *hex,
  uint8_t *bytes,
  size_t *byte_length,
  char *error,
  size_t error_size) {
  if (hex == NULL || bytes == NULL || byte_length == NULL) {
    snprintf(error, error_size, "Invalid message format");
    return false;
  }

  const size_t hex_length = strlen(hex);
  if (hex_length == 0 || (hex_length % 2) != 0) {
    snprintf(error, error_size, "The message must contain complete UTF-8 bytes");
    return false;
  }
  if (hex_length / 2 > MOTION_MAX_MESSAGE_BYTES) {
    snprintf(error, error_size, "Message exceeds the %d-byte limit", MOTION_MAX_MESSAGE_BYTES);
    return false;
  }

  for (size_t index = 0; index < hex_length; index += 2) {
    const int high = hex_value(hex[index]);
    const int low = hex_value(hex[index + 1]);
    if (high < 0 || low < 0) {
      snprintf(error, error_size, "The message contains invalid hexadecimal characters");
      return false;
    }
    bytes[index / 2] = (uint8_t)((high << 4) | low);
  }
  *byte_length = hex_length / 2;
  return true;
}

static void send_error(const char *code, const char *message) {
  char response[384];
  snprintf(
    response,
    sizeof(response),
    "{\"type\":\"error\",\"code\":\"%s\",\"message\":\"%s\"}",
    code,
    message);
  wb_robot_wwi_send_text(response);
}

static void send_ready(const MotionExecution *execution) {
  char response[640];
  snprintf(
    response,
    sizeof(response),
    "{\"type\":\"ready\",\"protocolVersion\":%d,\"maxBytes\":%d,"
    "\"defaultActionMs\":%d,\"minActionMs\":%d,\"maxActionMs\":%d,"
    "\"defaultIdleMs\":%d,\"minIdleMs\":%d,\"maxIdleMs\":%d,"
    "\"defaultSlotMs\":%d,\"minSlotMs\":%d,\"maxSlotMs\":%d,\"running\":%s}",
    MOTION_PROTOCOL_VERSION,
    MOTION_MAX_MESSAGE_BYTES,
    DEFAULT_ACTION_DURATION_MS,
    MIN_ACTION_DURATION_MS,
    MAX_ACTION_DURATION_MS,
    DEFAULT_IDLE_DURATION_MS,
    MIN_IDLE_DURATION_MS,
    MAX_IDLE_DURATION_MS,
    DEFAULT_ACTION_DURATION_MS,
    MIN_ACTION_DURATION_MS,
    MAX_ACTION_DURATION_MS,
    execution->running ? "true" : "false");
  wb_robot_wwi_send_text(response);
}

static void send_encoding(const MotionExecution *execution) {
  char response[1400];
  const size_t total_actions = execution->transmission.frame_count * MOTION_FRAME_ACTION_COUNT;
  const int cycle_duration_ms = execution->action_duration_ms + execution->idle_duration_ms;
  const double duration_seconds = total_actions * cycle_duration_ms / 1000.0;
  snprintf(
    response,
    sizeof(response),
    "{\"type\":\"encoding-start\",\"byteLength\":%zu,\"frameCount\":%zu,"
    "\"actionDurationMs\":%d,\"idleDurationMs\":%d,\"cycleDurationMs\":%d,"
    "\"totalActions\":%zu,\"durationSeconds\":%.3f}",
    execution->transmission.byte_length,
    execution->transmission.frame_count,
    execution->action_duration_ms,
    execution->idle_duration_ms,
    cycle_duration_ms,
    total_actions,
    duration_seconds);
  wb_robot_wwi_send_text(response);

  for (size_t index = 0; index < execution->transmission.frame_count; ++index) {
    const MotionEncodedFrame *frame = &execution->transmission.frames[index];
    snprintf(
      response,
      sizeof(response),
      "{\"type\":\"encoding-frame\",\"index\":%zu,\"seq\":%u,\"length\":%u,\"payloadHex\":\"%s\","
      "\"informationBits\":\"%s\",\"parityBits\":\"%s\",\"codewordBits\":\"%s\","
      "\"paddedBits\":\"%s\",\"actions\":\"%s\"}",
      index,
      frame->sequence,
      frame->payload_length,
      frame->payload_hex,
      frame->information_bits,
      frame->parity_bits,
      frame->codeword_bits,
      frame->padded_bits,
      frame->action_codes);
    wb_robot_wwi_send_text(response);
  }

  wb_robot_wwi_send_text("{\"type\":\"encoding-end\"}");
}

static void stop_execution(MotionExecution *execution, const char *reason) {
  if (!execution->running)
    return;
  execution->running = false;

  char response[256];
  snprintf(
    response,
    sizeof(response),
    "{\"type\":\"execution\",\"state\":\"stopped\",\"reason\":\"%s\"}",
    reason);
  wb_robot_wwi_send_text(response);
}

static bool parse_encode_command(
  const char *message,
  int *action_duration_ms,
  int *idle_duration_ms,
  uint8_t *bytes,
  size_t *byte_length,
  char *error,
  size_t error_size) {
  const char *first_separator = strchr(message, '|');
  if (first_separator == NULL) {
    snprintf(error, error_size, "The command is missing the slot duration");
    return false;
  }
  const char *second_separator = strchr(first_separator + 1, '|');
  if (second_separator == NULL) {
    snprintf(error, error_size, "The command is missing the message payload");
    return false;
  }
  const char *third_separator = strchr(second_separator + 1, '|');

  const size_t action_text_length = (size_t)(second_separator - first_separator - 1);
  if (action_text_length == 0 || action_text_length >= 16) {
    snprintf(error, error_size, "Invalid action duration");
    return false;
  }
  char action_text[16];
  memcpy(action_text, first_separator + 1, action_text_length);
  action_text[action_text_length] = '\0';

  char *action_end = NULL;
  const long parsed_action = strtol(action_text, &action_end, 10);
  if (action_end == action_text || *action_end != '\0' || parsed_action < MIN_ACTION_DURATION_MS ||
      parsed_action > MAX_ACTION_DURATION_MS) {
    snprintf(
      error,
      error_size,
      "The action duration must be between %d and %d ms",
      MIN_ACTION_DURATION_MS,
      MAX_ACTION_DURATION_MS);
    return false;
  }

  long parsed_idle = DEFAULT_IDLE_DURATION_MS;
  const char *hex_payload = second_separator + 1;
  if (third_separator != NULL) {
    const size_t idle_text_length = (size_t)(third_separator - second_separator - 1);
    if (idle_text_length == 0 || idle_text_length >= 16) {
      snprintf(error, error_size, "Invalid idle duration");
      return false;
    }
    char idle_text[16];
    memcpy(idle_text, second_separator + 1, idle_text_length);
    idle_text[idle_text_length] = '\0';

    char *idle_end = NULL;
    parsed_idle = strtol(idle_text, &idle_end, 10);
    if (idle_end == idle_text || *idle_end != '\0' || parsed_idle < MIN_IDLE_DURATION_MS ||
        parsed_idle > MAX_IDLE_DURATION_MS) {
      snprintf(
        error,
        error_size,
        "The idle duration must be between %d and %d ms",
        MIN_IDLE_DURATION_MS,
        MAX_IDLE_DURATION_MS);
      return false;
    }
    hex_payload = third_separator + 1;
  }

  if (!decode_hex_message(hex_payload, bytes, byte_length, error, error_size))
    return false;
  *action_duration_ms = (int)parsed_action;
  *idle_duration_ms = (int)parsed_idle;
  return true;
}

static void handle_robot_window_messages(
  MotionExecution *execution,
  double simulation_time,
  bool flight_stopped) {
  const char *message = NULL;
  while ((message = wb_robot_wwi_receive_text()) != NULL) {
    if (strcmp(message, "HELLO") == 0) {
      send_ready(execution);
      if (execution->has_encoding)
        send_encoding(execution);
      continue;
    }
    if (strcmp(message, "STOP") == 0) {
      stop_execution(execution, "user");
      continue;
    }

    const bool is_encode = strncmp(message, "ENCODE|", 7) == 0;
    const bool is_start = strncmp(message, "START|", 6) == 0;
    if (!is_encode && !is_start) {
      send_error("unknown-command", "Unrecognized Web UI command");
      continue;
    }
    if (execution->running) {
      send_error("busy", "A sequence is already running; stop it before starting another");
      continue;
    }
    if (is_start && flight_stopped) {
      send_error("flight-stopped", "Flight control is in emergency stop; press P to restore the motors");
      continue;
    }

    uint8_t bytes[MOTION_MAX_MESSAGE_BYTES];
    size_t byte_length = 0;
    int action_duration_ms = DEFAULT_ACTION_DURATION_MS;
    int idle_duration_ms = DEFAULT_IDLE_DURATION_MS;
    char error[160];
    if (!parse_encode_command(
          message,
          &action_duration_ms,
          &idle_duration_ms,
          bytes,
          &byte_length,
          error,
          sizeof(error)) ||
        !motion_encode_message(bytes, byte_length, &execution->transmission, error, sizeof(error))) {
      execution->has_encoding = false;
      send_error("invalid-input", error);
      continue;
    }

    execution->has_encoding = true;
    execution->action_duration_ms = action_duration_ms;
    execution->idle_duration_ms = idle_duration_ms;
    send_encoding(execution);

    if (is_start) {
      execution->running = true;
      execution->started_at = simulation_time;
      execution->last_status_at = -1.0;
      execution->last_reported_slot = (size_t)-1;
      execution->last_reported_action_phase = false;
      wb_robot_wwi_send_text("{\"type\":\"execution\",\"state\":\"running\"}");
    }
  }
}

static MotionAction current_automatic_action(
  MotionExecution *execution,
  double simulation_time,
  bool *action_phase_output) {
  if (action_phase_output != NULL)
    *action_phase_output = false;
  if (!execution->running)
    return MOTION_ACTION_HOVER;

  const double action_duration_seconds = execution->action_duration_ms / 1000.0;
  const double idle_duration_seconds = execution->idle_duration_ms / 1000.0;
  const double cycle_duration_seconds = action_duration_seconds + idle_duration_seconds;
  const double elapsed = simulation_time - execution->started_at;
  const size_t total_slots = execution->transmission.frame_count * MOTION_FRAME_ACTION_COUNT;
  const size_t global_slot = elapsed <= 0.0 ? 0 : (size_t)(elapsed / cycle_duration_seconds);
  if (global_slot >= total_slots) {
    execution->running = false;
    wb_robot_wwi_send_text("{\"type\":\"execution\",\"state\":\"complete\"}");
    return MOTION_ACTION_HOVER;
  }

  const size_t frame_index = global_slot / MOTION_FRAME_ACTION_COUNT;
  const size_t slot_index = global_slot % MOTION_FRAME_ACTION_COUNT;
  const MotionAction encoded_action = execution->transmission.frames[frame_index].actions[slot_index];
  const double cycle_elapsed = elapsed - global_slot * cycle_duration_seconds;
  const bool action_phase = cycle_elapsed < action_duration_seconds;
  if (action_phase_output != NULL)
    *action_phase_output = action_phase;

  if (execution->last_status_at < 0.0 || simulation_time - execution->last_status_at >= 0.1 ||
      execution->last_reported_slot != global_slot || execution->last_reported_action_phase != action_phase) {
    const double slot_progress = cycle_elapsed / cycle_duration_seconds;
    const double phase_progress = action_phase ? cycle_elapsed / action_duration_seconds :
                                                 (idle_duration_seconds > 0.0 ?
                                                    (cycle_elapsed - action_duration_seconds) / idle_duration_seconds :
                                                    1.0);
    char response[640];
    snprintf(
      response,
      sizeof(response),
      "{\"type\":\"status\",\"state\":\"running\",\"frameIndex\":%zu,\"frameCount\":%zu,"
      "\"slotIndex\":%zu,\"globalSlot\":%zu,\"totalSlots\":%zu,\"phase\":\"%s\","
      "\"encodedAction\":\"%c\",\"action\":\"%c\",\"actionName\":\"%s\","
      "\"slotProgress\":%.4f,\"phaseProgress\":%.4f,\"elapsedSeconds\":%.3f,\"totalSeconds\":%.3f}",
      frame_index,
      execution->transmission.frame_count,
      slot_index,
      global_slot,
      total_slots,
      action_phase ? "action" : "idle",
      motion_action_code(encoded_action),
      action_phase ? motion_action_code(encoded_action) : '-',
      action_phase ? motion_action_name(encoded_action) : "Idle / position hold",
      CLAMP(slot_progress, 0.0, 1.0),
      CLAMP(phase_progress, 0.0, 1.0),
      elapsed,
      total_slots * cycle_duration_seconds);
    wb_robot_wwi_send_text(response);
    execution->last_status_at = simulation_time;
    execution->last_reported_slot = global_slot;
    execution->last_reported_action_phase = action_phase;
  }

  return encoded_action;
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
  const double k_qe_tilt_angle = 8.0 * 3.14159265358979323846 / 180.0;
  const double k_move_tilt_angle = 5 * 3.14159265358979323846 / 180.0;
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
  MotionExecution execution = {
    .action_duration_ms = DEFAULT_ACTION_DURATION_MS,
    .idle_duration_ms = DEFAULT_IDLE_DURATION_MS,
  };

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
        stop_execution(&execution, "emergency");
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

    // The Webots robot window sends UTF-8 bytes as hexadecimal text. The C controller owns
    // authoritative BCH encoding and execution so the preview and the physical actions agree.
    handle_robot_window_messages(&execution, time, stopped);
    bool automatic_action_phase = false;
    MotionAction automatic_action = current_automatic_action(&execution, time, &automatic_action_phase);

    // Automatic protocol actions have exclusive control of horizontal motion. Altitude
    // controls and the P-key emergency stop remain available throughout a transmission.
    if (execution.running) {
      manual_roll_target = 0.0;
      manual_pitch_target = 0.0;
      yaw_disturbance = 0.0;
      q_is_pressed = false;
      e_is_pressed = false;
      if (automatic_action_phase) {
        // Match the existing keyboard driver: F = Up, H = Down, L = Q and R = E.
        // During the idle phase all movement targets stay at zero.
        if (automatic_action == MOTION_ACTION_FORWARD)
          manual_pitch_target = k_move_tilt_angle;
        else if (automatic_action == MOTION_ACTION_HOVER)
          manual_pitch_target = -k_move_tilt_angle;
        else if (automatic_action == MOTION_ACTION_MOVE_LEFT)
          manual_roll_target = -k_qe_tilt_angle;
        else if (automatic_action == MOTION_ACTION_MOVE_RIGHT)
          manual_roll_target = k_qe_tilt_angle;
      }
    }

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
