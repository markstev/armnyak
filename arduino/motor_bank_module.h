#ifndef ARMNYAK_ARDUINO_MOTOR_BANK_MODULE_H_
#define ARMNYAK_ARDUINO_MOTOR_BANK_MODULE_H_

#include <stdio.h>
#include <pb_decode.h>

#include "../arduinoio/lib/uc_module.h"
#include "../arduinoio/lib/message.h"
#include "motor_command.pb.h"
#include "motor.h"

namespace armnyak {

const int NUM_MOTORS = 3;

const char* MOTOR_INIT = "MINIT";
const int MOTOR_INIT_LENGTH = 5;
const char* MOTOR_UPDATE = "MUP";
const int MOTOR_UPDATE_LENGTH = 3;

class MotorBankModule : public arduinoio::UCModule {
 public:
  virtual const arduinoio::Message* Tick() {
    for (int i = 0; i < NUM_MOTORS; ++i) {
      motors_[i].Tick();
    }
    return NULL;
  }

  virtual bool AcceptMessage(const arduinoio::Message &message) {
    int length;
    const char* command = (const char*) message.command(&length);
    //char command[20]; length = 19;
    /*
command[0] = 'M';
command[1] = 'I';
command[2] = 'N';
command[3] = 'I';
command[4] = 'T';
command[5] = '\x08';
command[6] = '\x00';
command[7] = '\x10';
command[8] = '\r';
command[9] = '\x18';
command[10] = '\x08';
command[11] = ' ';
command[12] = '\t';
command[13] = '(';
command[14] = '\x0c';
command[15] = '0';
command[16] = '\x0b';
command[17] = '8';
command[18] = '\n';

buffer[0] = 8;
buffer[1] = 0;
buffer[2] = 16;
buffer[3] = 13;
buffer[4] = 24;
buffer[5] = 8;
buffer[6] = 32;
buffer[7] = 9;
buffer[8] = 40;
buffer[9] = 12;
buffer[10] = 48;
buffer[11] = 11;
buffer[12] = 56;
buffer[13] = 10;
//Command: ['M', 'I', 'N', 'I', 'T', '\x08', '\x00', '\x10', '\r', '\x18', '\x08', ' ', '\t', '(', '\x0c', '0', '\x0b', '8', '\n']
//Command: [77, 73, 78, 73, 84, 8, 0, 16, 13, 24, 8, 32, 9, 40, 12, 48, 11, 56, 10]
*/
    if (length > MOTOR_INIT_LENGTH &&
        (strncmp(command, MOTOR_INIT, MOTOR_INIT_LENGTH) == 0)) {
      const uint8_t* buffer = (const uint8_t*) (command + MOTOR_INIT_LENGTH);
      MotorInitProto command_proto = MotorInitProto_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(buffer, length - MOTOR_INIT_LENGTH - 1);
      const bool status = pb_decode(&stream, MotorInitProto_fields, &command_proto);
      if (!status) return false;
      if (command_proto.address >= NUM_MOTORS) return false;
      motors_[command_proto.address].Init(command_proto);
    } else if (length > MOTOR_UPDATE_LENGTH &&
        (strncmp(command, MOTOR_UPDATE, MOTOR_UPDATE_LENGTH) == 0)) {
      const uint8_t* buffer = (const uint8_t*) (command + MOTOR_UPDATE_LENGTH);
//    uint8_t buffer[23];
//buffer[0] = 8;
//buffer[1] = 0;
//buffer[2] = 16;
//buffer[3] = 0;
//buffer[4] = 29;
//buffer[5] = 0;
//buffer[6] = 0;
//buffer[7] = 0;
//buffer[8] = 63;
//buffer[9] = 37;
//buffer[10] = 0;
//buffer[11] = 0;
//buffer[12] = 0;
//buffer[13] = 0;
//buffer[14] = 40;
//buffer[15] = 143;
//buffer[16] = 78;
//buffer[17] = 48;
//buffer[18] = 1;

      MotorMoveProto command_proto = MotorMoveProto_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(buffer, length - MOTOR_UPDATE_LENGTH - 1);
      const bool status = pb_decode(&stream, MotorMoveProto_fields, &command_proto);
      if (!status) return false;
      if (command_proto.address >= NUM_MOTORS) return false;
      motors_[command_proto.address].Update(command_proto);
    }
  }

 private:
  Motor motors_[NUM_MOTORS];
};

}  // namespace armnyak

#endif  // ARMNYAK_ARDUINO_MOTOR_BANK_MODULE_H_
