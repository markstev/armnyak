#!/usr/bin/python

from motor2 import MotorBankBase
from protoc.motor_command_pb2 import MotorMoveProto
import time

stepper_dir_pin = 8
stepper_pulse_pin = 9
forward = True
final_wait = 1000
max_wait = 4000
temp_pin_threshold = 20

bank= MotorBankBase()
#   for i in range(10):
#     forward = not forward
#     steps = 400
#     bank.base_motor.SetMicrostepDivision(1)
#     move_proto = MotorMoveProto()
#     move_proto.address = 0
#     move_proto.direction = forward
#     move_proto.max_speed = 1.0 / (i + 1)
#     move_proto.min_speed = 10000.0
#     move_proto.steps = steps
#     move_proto.disable_after_moving = True
#     bank.base_motor.Move(move_proto)
#     time.sleep(1)

for i in range(2):
  forward = not forward
  steps = 400
  bank.base_motor.SetMicrostepDivision(1)
  move_proto = MotorMoveProto()
  move_proto.address = 0
  move_proto.direction = forward
  move_proto.max_speed = 0.5
  move_proto.min_speed = 0.0
  move_proto.steps = steps
  move_proto.disable_after_moving = True
  bank.base_motor.Move(move_proto)
  #time.sleep(1)
  bank.wrist_motor.SetMicrostepDivision(1)

  move_proto = MotorMoveProto()
  move_proto.address = 1
  move_proto.direction = not forward
  move_proto.max_speed = 0.1
  move_proto.min_speed = 0.0
  move_proto.steps = steps / 3
  move_proto.disable_after_moving = True
  bank.wrist_motor.Move(move_proto)
  time.sleep(2)
