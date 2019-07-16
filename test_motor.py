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

bank= MotorBankBase()
start = time.time()
bank.wrist_motor.Configure(microsteps=4, max_steps=1600, min_steps=-1600)
bank.base_motor.Configure(microsteps=2, max_steps=32000, min_steps=-32000)
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
    bank.wrist_motor.Move(move_proto)


def calibrate_wrist(bank):
    forward = True
    move_proto = MotorMoveProto()
    for i in range(8, 10):
      forward = not forward
      steps = 4
      move_proto.address = 1
      move_proto.direction = forward
      move_proto.max_speed = (i + 1) * 0.1
      logging.info(move_proto.max_speed)
      move_proto.min_speed = 0.0
      move_proto.steps = steps
      move_proto.disable_after_moving = True
      move_proto.use_absolute_steps = True
      if forward:
          # 1300 steps = a quarter turn in the wrist (microstepping x 4)
          move_proto.absolute_steps = 1300
      else:
          move_proto.absolute_steps = -1300
      bank.wrist_motor.Move(move_proto)
      # Can do almost a half turn in 1s at max speed (microstepping x 4)
      time.sleep(1)
    move_proto.absolute_steps = 0
    move_proto.disable_after_moving = True
    move_proto.use_absolute_steps = True
    bank.wrist_motor.Move(move_proto)

#calibrate_wrist(bank)
#calibrate_base(bank)

#bank.wrist_motor.MoveRelative(0.5)
bank.wrist_motor.MoveAbsolute(0.6, math.pi / 4)
time.sleep(3)
bank.wrist_motor.MoveAbsolute(0.6, -math.pi / 4)
#bank.wrist_motor.MoveRelative(-0.5)

### for i in range(2):
###   forward = not forward
###   steps = 400
###   bank.base_motor.SetMicrostepDivision(1)
###   move_proto = MotorMoveProto()
###   move_proto.address = 0
###   move_proto.direction = forward
###   move_proto.max_speed = 0.5
###   move_proto.min_speed = 0.0
###   move_proto.steps = steps
###   move_proto.disable_after_moving = True
###   bank.base_motor.Move(move_proto)
###   #time.sleep(1)
###   bank.wrist_motor.SetMicrostepDivision(1)

###   move_proto = MotorMoveProto()
###   move_proto.address = 1
###   move_proto.direction = not forward
###   move_proto.max_speed = 0.1
###   move_proto.min_speed = 0.0
###   move_proto.steps = steps / 3
###   move_proto.disable_after_moving = True
###   bank.wrist_motor.Move(move_proto)
###   time.sleep(2)
