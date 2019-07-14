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
const int MOTOR_INIT_LENGTH = 4;
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
    if (length > MOTOR_INIT_LENGTH &&
        (strncmp(command, MOTOR_INIT, MOTOR_INIT_LENGTH) == 0)) {
      const uint8_t* buffer = (const uint8_t*) (command + MOTOR_INIT_LENGTH);
      MotorInitProto command_proto = MotorInitProto_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(buffer, length);
      const bool status = pb_decode(&stream, MotorInitProto_fields, &command_proto);
      if (!status) return false;
      if (command_proto.address >= NUM_MOTORS) return false;
      motors_[command_proto.address].Init(command_proto);
    }
  }

 private:
  Motor motors_[NUM_MOTORS];
};

}  // namespace armnyak

#endif  // ARMNYAK_ARDUINO_MOTOR_BANK_MODULE_H_
