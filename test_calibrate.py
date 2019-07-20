from motor2 import MotorBankBase
from protoc.motor_command_pb2 import MotorMoveProto
import time
import logging
import math
from io_read import InputBoard

bank = MotorBankBase()
input_board = InputBoard()
input_board.Start()
def calibrate_base(bank, input_board):
  forward = True
  for motor in [bank.base_motor, bank.base_motor2]:
      motor.Configure(microsteps=1, max_steps=600000, min_steps=-600000)
      motor.MoveAbsolute(0.6, forward * math.pi * 1)
  def stop_all():
      for motor in [bank.base_motor, bank.base_motor2]:
          motor.MoveRelative(0.0)
      logging.info("Stopping!")

  input_board.RegisterCallback(26, False, stop_all, permanent=True)

def SetupTares(bank, input_board):
  def test_tare():
      for motor in [bank.base_motor, bank.base_motor2]:
          motor.Tare(-math.pi)
  input_board.RegisterCallback(26, False, test_tare, permanent=True)

SetupTares(bank, input_board)
#calibrate_base(bank, input_board)
forward = 1.0
for i in range(20):
  forward *= -1.0
  for motor in [bank.base_motor, bank.base_motor2]:
      motor.Configure(microsteps=1, max_steps=600000, min_steps=-600000)
      motor.MoveAbsolute(0.6, forward * math.pi / 6)
      time.sleep(4)
    #logging.info("P24=%d P25=%d P26=%d", input_board.GetPin(24), input_board.GetPin(25), input_board.GetPin(26))
