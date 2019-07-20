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
const char* MOTOR_CONFIG = "MCONF";
const int MOTOR_CONFIG_LENGTH = 5;
const char* MOTOR_TARE = "MTARE";
const int MOTOR_TARE_LENGTH = 5;
const char* MOTOR_UPDATE_ALL = "MUPA";
const int MOTOR_UPDATE_ALL_LENGTH = 4;

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
      pb_istream_t stream = pb_istream_from_buffer(buffer, length - MOTOR_INIT_LENGTH - 1);
      const bool status = pb_decode(&stream, MotorInitProto_fields, &command_proto);
      if (!status) return false;
      if (command_proto.address >= NUM_MOTORS) return false;
      motors_[command_proto.address].Init(command_proto);
    } else if (length > MOTOR_UPDATE_LENGTH &&
        (strncmp(command, MOTOR_UPDATE, MOTOR_UPDATE_LENGTH) == 0)) {
      const uint8_t* buffer = (const uint8_t*) (command + MOTOR_UPDATE_LENGTH);

      MotorMoveProto command_proto = MotorMoveProto_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(buffer, length - MOTOR_UPDATE_LENGTH - 1);
      const bool status = pb_decode(&stream, MotorMoveProto_fields, &command_proto);
      if (!status) return false;
      UpdateMotor(command_proto);
    } else if (length > MOTOR_CONFIG_LENGTH &&
        (strncmp(command, MOTOR_CONFIG, MOTOR_CONFIG_LENGTH) == 0)) {
      const uint8_t* buffer = (const uint8_t*) (command + MOTOR_CONFIG_LENGTH);

      MotorConfigProto command_proto = MotorConfigProto_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(buffer, length - MOTOR_CONFIG_LENGTH - 1);
      const bool status = pb_decode(&stream, MotorConfigProto_fields, &command_proto);
      if (!status) return false;
      if (command_proto.address >= NUM_MOTORS) return false;
      motors_[command_proto.address].Config(command_proto);
    } else if (length > MOTOR_TARE_LENGTH &&
        (strncmp(command, MOTOR_TARE, MOTOR_TARE_LENGTH) == 0)) {
      const uint8_t* buffer = (const uint8_t*) (command + MOTOR_TARE_LENGTH);

      MotorTareProto command_proto = MotorTareProto_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(buffer, length - MOTOR_TARE_LENGTH - 1);
      const bool status = pb_decode(&stream, MotorTareProto_fields, &command_proto);
      if (!status) return false;
      if (command_proto.address >= NUM_MOTORS) return false;
      motors_[command_proto.address].Tare(command_proto.tare_to_steps);
    } else if (length > MOTOR_UPDATE_ALL_LENGTH &&
        (strncmp(command, MOTOR_UPDATE_ALL, MOTOR_UPDATE_ALL_LENGTH) == 0)) {
      const uint8_t* buffer = (const uint8_t*) (command + MOTOR_UPDATE_ALL_LENGTH);

      MotorMoveAllProto command_proto = MotorMoveAllProto_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(buffer, length - MOTOR_UPDATE_ALL_LENGTH - 1);
      const bool status = pb_decode(&stream, MotorMoveAllProto_fields, &command_proto);
      if (!status) return false;
      UpdateMotor(command_proto.top_left);
      UpdateMotor(command_proto.middle);
      UpdateMotor(command_proto.top_right);
    }
    return true;
  }

  void UpdateMotor(const MotorMoveProto &update_proto) {
    if (update_proto.address >= NUM_MOTORS) return;
    motors_[update_proto.address].Update(update_proto);
  }

 private:
  Motor motors_[NUM_MOTORS];
};

}  // namespace armnyak

#endif  // ARMNYAK_ARDUINO_MOTOR_BANK_MODULE_H_
