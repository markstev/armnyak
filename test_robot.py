#!/usr/bin/python

from motor2 import MotorBankBase
from io_read import InputBoard
from config import ArmConfig
from protoc.motor_command_pb2 import MotorMoveProto
from hand import Hand
from arm import Arm
import time
import logging
import math

bank = MotorBankBase()
io_reader = InputBoard()
io_reader.Start()
arm_config = ArmConfig()
arm_config.ApplyTares(bank, io_reader)
#   while True:
#       s = "\n------\n"
#       for i in range(22, 32, 2):
#           s += " PIN %d = %d\n" % (i, io_reader.GetPin(i))

#       logging.info("RPS = %f %s", io_reader.ReadsPerSec(), s)
#       time.sleep(0.2)
bank.Release()
bank.Rezero()
time.sleep(5.0)

hand = Hand(bank, io_reader, arm_config)
arm = Arm(bank, io_reader, arm_config)
arm.Calibrate()
arm.GoToPositionHeight()
arm.LowerOntoBottle()
#arm.Lower()
#arm.Rotate()
hand.Grab()
#   #hand.Hold()
time.sleep(2.0)
arm.RaiseFully()
hand.Dispense()
time.sleep(1.0)

arm.SetupLowerStop()
bank.Rezero()
bank.Release()
bank.Rezero()
time.sleep(15.0)
#   #hand.Hold()
