#include "motion_protocol.h"

#include <stdio.h>
#include <string.h>

#define BCH_GENERATOR 0x782CFULL
#define BCH_PARITY_BITS 18

static const MotionAction k_sync_actions[MOTION_SYNC_ACTION_COUNT] = {
  MOTION_ACTION_MOVE_RIGHT,
  MOTION_ACTION_MOVE_LEFT,
  MOTION_ACTION_HOVER,
  MOTION_ACTION_MOVE_RIGHT,
  MOTION_ACTION_FORWARD,
  MOTION_ACTION_MOVE_LEFT,
  MOTION_ACTION_FORWARD,
  MOTION_ACTION_HOVER,
};

static uint64_t polynomial_remainder(uint64_t value) {
  for (int bit = 62; bit >= BCH_PARITY_BITS; --bit) {
    if ((value & (1ULL << bit)) != 0)
      value ^= BCH_GENERATOR << (bit - BCH_PARITY_BITS);
  }
  return value & ((1ULL << BCH_PARITY_BITS) - 1ULL);
}

static void format_bits(uint64_t value, int width, char *output) {
  for (int bit = width - 1; bit >= 0; --bit)
    output[width - 1 - bit] = (value & (1ULL << bit)) != 0 ? '1' : '0';
  output[width] = '\0';
}

char motion_action_code(MotionAction action) {
  switch (action) {
    case MOTION_ACTION_HOVER:
      return 'H';
    case MOTION_ACTION_FORWARD:
      return 'F';
    case MOTION_ACTION_MOVE_LEFT:
      return 'L';
    case MOTION_ACTION_MOVE_RIGHT:
      return 'R';
    default:
      return '?';
  }
}

const char *motion_action_name(MotionAction action) {
  switch (action) {
    case MOTION_ACTION_HOVER:
      return "Backward";
    case MOTION_ACTION_FORWARD:
      return "Forward";
    case MOTION_ACTION_MOVE_LEFT:
      return "Move left";
    case MOTION_ACTION_MOVE_RIGHT:
      return "Move right";
    default:
      return "Unknown";
  }
}

bool motion_codeword_is_valid(uint64_t codeword) {
  return (codeword >> 63) == 0 && polynomial_remainder(codeword) == 0;
}

bool motion_encode_frame(const uint8_t *payload, size_t payload_length, uint8_t sequence, MotionEncodedFrame *frame) {
  if (frame == NULL || payload_length > MOTION_PAYLOAD_BYTES_PER_FRAME || sequence > 3 ||
      (payload_length > 0 && payload == NULL))
    return false;

  memset(frame, 0, sizeof(*frame));
  frame->sequence = sequence;
  frame->payload_length = (uint8_t)payload_length;
  if (payload_length > 0)
    memcpy(frame->payload, payload, payload_length);

  uint64_t data = 0;
  for (size_t index = 0; index < MOTION_PAYLOAD_BYTES_PER_FRAME; ++index)
    data = (data << 8) | frame->payload[index];

  frame->information = ((uint64_t)payload_length << 42) | ((uint64_t)sequence << 40) | data;
  const uint64_t shifted_information = frame->information << BCH_PARITY_BITS;
  const uint64_t parity = polynomial_remainder(shifted_information);
  frame->codeword = shifted_information | parity;
  frame->padded_codeword = frame->codeword << 1;

  memcpy(frame->actions, k_sync_actions, sizeof(k_sync_actions));
  for (size_t symbol_index = 0; symbol_index < MOTION_DATA_ACTION_COUNT; ++symbol_index) {
    const int shift = 62 - (int)(symbol_index * 2);
    const uint8_t symbol = (uint8_t)((frame->padded_codeword >> shift) & 0x3ULL);
    MotionAction action = MOTION_ACTION_HOVER;
    if (symbol == 1)
      action = MOTION_ACTION_FORWARD;
    else if (symbol == 2)
      action = MOTION_ACTION_MOVE_RIGHT;
    else if (symbol == 3)
      action = MOTION_ACTION_MOVE_LEFT;
    frame->actions[MOTION_SYNC_ACTION_COUNT + symbol_index] = action;
  }

  for (size_t index = 0; index < payload_length; ++index)
    snprintf(frame->payload_hex + index * 2, sizeof(frame->payload_hex) - index * 2, "%02X", frame->payload[index]);
  frame->payload_hex[payload_length * 2] = '\0';
  format_bits(frame->information, 45, frame->information_bits);
  format_bits(parity, 18, frame->parity_bits);
  format_bits(frame->codeword, 63, frame->codeword_bits);
  format_bits(frame->padded_codeword, 64, frame->padded_bits);
  for (size_t index = 0; index < MOTION_FRAME_ACTION_COUNT; ++index)
    frame->action_codes[index] = motion_action_code(frame->actions[index]);
  frame->action_codes[MOTION_FRAME_ACTION_COUNT] = '\0';

  return motion_codeword_is_valid(frame->codeword);
}

bool motion_encode_message(
  const uint8_t *message,
  size_t message_length,
  MotionTransmission *transmission,
  char *error,
  size_t error_size) {
  if (transmission == NULL || (message_length > 0 && message == NULL)) {
    if (error != NULL && error_size > 0)
      snprintf(error, error_size, "Invalid encoder input");
    return false;
  }
  if (message_length == 0) {
    if (error != NULL && error_size > 0)
      snprintf(error, error_size, "Enter at least one UTF-8 byte");
    return false;
  }
  if (message_length > MOTION_MAX_MESSAGE_BYTES) {
    if (error != NULL && error_size > 0)
      snprintf(error, error_size, "Message exceeds the %d-byte limit", MOTION_MAX_MESSAGE_BYTES);
    return false;
  }

  memset(transmission, 0, sizeof(*transmission));
  transmission->byte_length = message_length;
  transmission->frame_count = (message_length + MOTION_PAYLOAD_BYTES_PER_FRAME - 1) / MOTION_PAYLOAD_BYTES_PER_FRAME;

  for (size_t frame_index = 0; frame_index < transmission->frame_count; ++frame_index) {
    const size_t offset = frame_index * MOTION_PAYLOAD_BYTES_PER_FRAME;
    const size_t remaining = message_length - offset;
    const size_t frame_length = remaining < MOTION_PAYLOAD_BYTES_PER_FRAME ? remaining : MOTION_PAYLOAD_BYTES_PER_FRAME;
    if (!motion_encode_frame(message + offset, frame_length, (uint8_t)(frame_index % 4), &transmission->frames[frame_index])) {
      if (error != NULL && error_size > 0)
        snprintf(error, error_size, "Failed to encode frame %zu", frame_index + 1);
      memset(transmission, 0, sizeof(*transmission));
      return false;
    }
  }

  if (error != NULL && error_size > 0)
    error[0] = '\0';
  return true;
}
