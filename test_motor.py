#!/usr/bin/python

from motor2 import MotorBankBase
from protoc.motor_command_pb2 import MotorMoveProto
import time
import logging
import math

stepper_dir_pin = 8
stepper_pulse_pin = 9
final_wait = 1000
max_wait = 4000
temp_pin_threshold = 20

bank = MotorBankBase()
start = time.time()
#bank.wrist_motor.Configure(microsteps=1, max_steps=1600, min_steps=-1600)
#bank.base_motor.Configure(microsteps=1, max_steps=32000, min_steps=-32000)
stop = time.time()
logging.info("msec to push update = %f", (stop - start) * 1000.0)
def calibrate_base(bank):
    forward = True
    move_proto = MotorMoveProto()
    for i in range(4, 10):
      forward = not forward
      steps = 20000
      move_proto.address = 0
      move_proto.direction = forward
      move_proto.max_speed = (i + 1) * 0.1
      logging.info(move_proto.max_speed)
      move_proto.min_speed = 0.0
      move_proto.steps = steps
      move_proto.disable_after_moving = False
      move_proto.use_absolute_steps = False
      if forward:
          # 2000 steps is roughly 12% of a rotation, microstepping x 2
          move_proto.absolute_steps = 2000
      else:
          move_proto.absolute_steps = -2000
      bank.base_motor.Move(move_proto)
      # About 6s for a full rotation
      time.sleep(10)
    move_proto.absolute_steps = 0
    move_proto.disable_after_moving = False
    move_proto.use_absolute_steps = True
    bank.base_motor.Move(move_proto)


def calibrate_motor(bank, motor, angle):
    forward = True
    for i in range(4, 10):
      logging.info("Move %d", i)
      if forward:
        motor.MoveAbsolute(i * 0.1, angle)
      else:
        motor.MoveAbsolute(i * 0.1, -angle)
      bank.WaitDone()
    motor.SetDisableAfterMoving(True)
    motor.MoveAbsolute(0.6, 0)
    bank.WaitDone()
    time.sleep(0.5)


calibrate_motor(bank, bank.lift_motor, math.pi * 0.5)
