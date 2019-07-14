
#include <stdio.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include "motor_command.pb.h"

int main(int argc, char **argv) {
  uint8_t buffer[20];
  size_t message_length;
  {
    MotorInitProto message = MotorInitProto_init_zero;
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    message.address = 82;
    message.step_pin = 12;
    const bool status = pb_encode(&stream, MotorInitProto_fields, &message);
    if (!status) {
      return 1;
    }
  }
  {
    MotorInitProto command_proto = MotorInitProto_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(buffer, message_length);
    const bool status = pb_decode(&stream, MotorInitProto_fields, &command_proto);
    if (!status) {
      return 2;
    }
  }
  return 0;
}
