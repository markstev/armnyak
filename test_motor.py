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
for i in range(10):
  forward = not forward
  steps = 2000
  move_proto = MotorMoveProto()
  move_proto.address = 0
  move_proto.direction = forward
  move_proto.max_speed = i * 0.1
  move_proto.min_speed = 0.0
  move_proto.steps = steps
  move_proto.disable_after_moving = True
  bank.base_motor.Move(move_proto)
  time.sleep(4)
