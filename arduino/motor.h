#ifndef ARMNYAK_ARDUINO_MOTOR_H_
#define ARMNYAK_ARDUINO_MOTOR_H_

#include "motor_command.pb.h"
#include <stdint.h>

namespace armnyak {
namespace {
const double kSpeedUpFactor = 0.996;
const double kSlowDownFactor = 1.004;

const int kPhysicalMinWait = 200;
const int kPhysicalMaxWait = 2000;

bool TimeToSlowDown(const int steps_remaining, const int current_wait, const int max_wait) {
  const int kSlowDownIncrement = 100;
  const double kSlowDownFactorIncrement = 1.49063488565;
  const int steps_increment = steps_remaining / kSlowDownIncrement;
  double new_wait = current_wait;
  for (int i = 0; i < steps_increment; ++i) {
    new_wait *= kSlowDownFactorIncrement;
    if (new_wait > max_wait) {
      // We can wait and still slow down in time.
      return false;
    }
  }
  return true;
}

int SpeedToWaitTime(const float speed) {
  const int range = kPhysicalMaxWait - kPhysicalMinWait;
  return range * (1.0 - speed) + kPhysicalMinWait;
}

}  // namespace

class Motor {
 public:
  Motor() {
   remaining_steps_ = 0;
   pulse_state_ = false;
  }
  
  void Init(const MotorInitProto &init_proto) {
    init_proto_ = init_proto;
    pinMode(init_proto_.enable_pin, OUTPUT);
    pinMode(init_proto_.dir_pin, OUTPUT);
    pinMode(init_proto_.step_pin, OUTPUT);
    pinMode(init_proto_.ms0_pin, OUTPUT);
    pinMode(init_proto_.ms1_pin, OUTPUT);
    pinMode(init_proto_.ms2_pin, OUTPUT);
    digitalWrite(init_proto_.enable_pin, HIGH);
  }

  uint32_t address() const {
    return init_proto_.address;
  }

  void Update(const MotorMoveProto &move_proto) {
    digitalWrite(init_proto_.dir_pin, move_proto.direction);
    max_wait_ = SpeedToWaitTime(move_proto.min_speed);
    min_wait_ = SpeedToWaitTime(move_proto.max_speed);
    current_wait_ = max_wait_;
    remaining_steps_ = move_proto.steps;
    disable_after_moving_ = move_proto.disable_after_moving;
    next_step_in_usec_ = 0;
    digitalWrite(init_proto_.enable_pin, LOW);
  }

  void Tick() {
    if (remaining_steps_ <= 0) return;
    const unsigned long now = micros();
    if (now < next_step_in_usec_) return;
    Step();
    UpdateWait(now);
    --remaining_steps_;
    if (remaining_steps_ == 0 && disable_after_moving_) {
      digitalWrite(init_proto_.enable_pin, HIGH);
    }
  }

  void Step() {
    pulse_state_ = !pulse_state_;
    digitalWrite(init_proto_.step_pin, pulse_state_ ? HIGH : LOW);
  }

  void UpdateWait(unsigned long now) {
    if (TimeToSlowDown(remaining_steps_, current_wait_, max_wait_)) {
      current_wait_ *= kSlowDownFactor;
    } else if (current_wait_ > min_wait_) {
      current_wait_ *= kSpeedUpFactor;
    }
    next_step_in_usec_ = now + current_wait_;
  }

 private:
  MotorInitProto init_proto_;

  // Stepping state
  bool pulse_state_;
  uint32_t current_wait_;
  uint32_t max_wait_;
  uint32_t min_wait_;
  unsigned long next_step_in_usec_;
  uint32_t remaining_steps_;
  bool disable_after_moving_;
};

}  // namespace armnyak

#endif  // ARMNYAK_ARDUINO_MOTOR_H_
