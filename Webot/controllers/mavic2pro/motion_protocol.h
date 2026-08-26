#ifndef MOTION_PROTOCOL_H
#define MOTION_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MOTION_PROTOCOL_VERSION 3
#define MOTION_PHYSICAL_PROFILE_VERSION 5
#define MOTION_PAYLOAD_BYTES_PER_FRAME 5
#define MOTION_SYNC_ACTION_COUNT 8
#define MOTION_DATA_ACTION_COUNT 32
#define MOTION_FRAME_ACTION_COUNT (MOTION_SYNC_ACTION_COUNT + MOTION_DATA_ACTION_COUNT)
#define MOTION_MAX_MESSAGE_BYTES 240
#define MOTION_MAX_FRAMES ((MOTION_MAX_MESSAGE_BYTES + MOTION_PAYLOAD_BYTES_PER_FRAME - 1) / MOTION_PAYLOAD_BYTES_PER_FRAME)

typedef enum {
  MOTION_ACTION_HOVER = 0,
  MOTION_ACTION_FORWARD = 1,
  MOTION_ACTION_MOVE_LEFT = 2,
  MOTION_ACTION_MOVE_RIGHT = 3
} MotionAction;

typedef struct {
  uint8_t sequence;
  uint8_t payload_length;
  uint8_t payload[MOTION_PAYLOAD_BYTES_PER_FRAME];
  uint64_t information;
  uint64_t codeword;
  uint64_t padded_codeword;
  MotionAction actions[MOTION_FRAME_ACTION_COUNT];
  char payload_hex[MOTION_PAYLOAD_BYTES_PER_FRAME * 2 + 1];
  char information_bits[46];
  char parity_bits[19];
  char codeword_bits[64];
  char padded_bits[65];
  char action_codes[MOTION_FRAME_ACTION_COUNT + 1];
} MotionEncodedFrame;

typedef struct {
  size_t byte_length;
  size_t frame_count;
  MotionEncodedFrame frames[MOTION_MAX_FRAMES];
} MotionTransmission;

bool motion_encode_frame(const uint8_t *payload, size_t payload_length, uint8_t sequence, MotionEncodedFrame *frame);
bool motion_encode_message(
  const uint8_t *message,
  size_t message_length,
  MotionTransmission *transmission,
  char *error,
  size_t error_size);
bool motion_codeword_is_valid(uint64_t codeword);
const char *motion_action_name(MotionAction action);
char motion_action_code(MotionAction action);

#endif
