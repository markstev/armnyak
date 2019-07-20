from motor2 import MotorBankBase
from protoc.motor_command_pb2 import MotorMoveProto
import time
import logging
import math
from io_read import InputBoard

bank = MotorBankBase()

#calibrate_base(bank, input_board)
forward = 1.0
for i in range(20):
  forward *= -1.0
  for motor in [bank.base_motor, bank.base_motor2]:
      motor.Configure(microsteps=1, max_steps=600000, min_steps=-600000)
      move_proto = motor.MoveAbsoluteProto(0.6, forward * math.pi / 6)
      bank.WriteMany([move_proto, move_proto, move_proto], motor)
      time.sleep(4)
    #logging.info("P24=%d P25=%d P26=%d", input_board.GetPin(24), input_board.GetPin(25), input_board.GetPin(26))
