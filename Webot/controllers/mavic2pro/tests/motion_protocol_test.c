#include "../motion_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_hello_reference_vector(void) {
  static const uint8_t hello[] = {'H', 'e', 'l', 'l', 'o'};
  MotionEncodedFrame frame;

  assert(motion_encode_frame(hello, sizeof(hello), 0, &frame));
  assert(strcmp(frame.information_bits, "101000100100001100101011011000110110001101111") == 0);
  assert(strcmp(frame.parity_bits, "100100101101010011") == 0);
  assert(strcmp(frame.codeword_bits, "101000100100001100101011011000110110001101111100100101101010011") == 0);
  assert(strcmp(frame.padded_bits, "1010001001000011001010110110001101100011011111001001011010100110") == 0);
  assert(strcmp(frame.action_codes, "RLHRFLFHRRFRLFFHFRRHLRFHLRFHLHHFRLLRRRLR") == 0);
  assert(motion_codeword_is_valid(frame.codeword));
}

static void test_message_segmentation(void) {
  static const uint8_t message[] = "Hello world";
  MotionTransmission transmission;
  char error[128];

  assert(motion_encode_message(message, sizeof(message) - 1, &transmission, error, sizeof(error)));
  assert(transmission.byte_length == 11);
  assert(transmission.frame_count == 3);
  assert(transmission.frames[0].sequence == 0 && transmission.frames[0].payload_length == 5);
  assert(transmission.frames[1].sequence == 1 && transmission.frames[1].payload_length == 5);
  assert(transmission.frames[2].sequence == 2 && transmission.frames[2].payload_length == 1);
  assert(strcmp(transmission.frames[0].payload_hex, "48656C6C6F") == 0);
  assert(strcmp(transmission.frames[1].payload_hex, "20776F726C") == 0);
  assert(strcmp(transmission.frames[2].payload_hex, "64") == 0);
}

static void test_input_validation(void) {
  MotionTransmission transmission;
  char error[128];
  uint8_t oversized[MOTION_MAX_MESSAGE_BYTES + 1] = {0};

  assert(!motion_encode_message(NULL, 0, &transmission, error, sizeof(error)));
  assert(error[0] != '\0');
  assert(!motion_encode_message(oversized, sizeof(oversized), &transmission, error, sizeof(error)));
  assert(error[0] != '\0');
}

int main(void) {
  test_hello_reference_vector();
  test_message_segmentation();
  test_input_validation();
  puts("motion_protocol_test: all tests passed");
  return 0;
}
