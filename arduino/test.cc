
//#include <stdio.h>
//#include <pb_decode.h>
//#include <pb_encode.h>
//#include "motor_command.pb.h"

//int main (int argc, char *argv[]) {
//  printf("Hello, world!\n");
//  uint8_t buffer[20];
//  size_t message_length;
//  {
//    MotorInitProto message = MotorInitProto_init_zero;
//    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
//    message.address = 0;
//    message.enable_pin = 13;
//    message.step_pin = 9;
//    message.dir_pin = 8;
//    message.ms0_pin = 12;
//    message.ms1_pin = 11;
//    message.ms2_pin = 10;
//    const bool status = pb_encode(&stream, MotorInitProto_fields, &message);
//    if (!status) {
//      return 1;
//    }
//    for (int i = 0; i < stream.bytes_written; ++i) {
//      printf("Byte: %d of %d: %d\n", i, stream.bytes_written, buffer[i]);
//    }
//  }
//  {
//    MotorInitProto command_proto = MotorInitProto_init_zero;
//    pb_istream_t stream = pb_istream_from_buffer(buffer, message_length);
//    const bool status = pb_decode(&stream, MotorInitProto_fields, &command_proto);
//    if (!status) {
//      return 2;
//    }
//  }
//  fflush(stdout);
//  return 0;
//}
