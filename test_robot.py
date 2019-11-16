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
#hand.Dispense()
hand.Calibrate()
#bank.WaitDone()
bank.base_motor.SetDisableAfterMoving(False)
arm.Rotate(-math.pi / 2)
bank.WaitDone()
arm.Calibrate()
bank.WaitDone()
#arm.RaiseFully()
#bank.WaitDone()
arm.GoToPositionHeight()
bank.WaitDone()
arm.Rotate(0.0)
bank.WaitDone()
hand.Release()
bank.WaitDone()
time.sleep(5.0)
#arm.Rotate(math.pi / 30)
bank.WaitDone()
arm.LowerOntoBottle()
bank.WaitDone()
#   #   #arm.Lower()
#   #   #arm.Rotate()
hand.Grab()
bank.WaitDone()
#   #   #   #hand.Hold()
#   #   time.sleep(2.0)
arm.RaiseFully()
bank.WaitDone()
hand.Dispense()
bank.WaitDone()
arm.SetWristToGrabPosition()
#   #   #   time.sleep(1.0)
#   #   bank.WaitDone()

#   #   arm.SetupLowerStop()
#   #   bank.Rezero()
arm.Lower()
bank.WaitDone()
#arm.Rotate(-math.pi / 25)
#arm.Rotate(math.pi / 30)
#bank.WaitDone()
hand.Release()
bank.WaitDone()
#arm.GoToReleasePositionHeight()
#bank.WaitDone()
#hand.Rotate(math.pi / 6)
arm.RaiseOverDroppedBottle()
bank.WaitDone()
logging.info("Raise fully")
arm.RaiseFully()
bank.WaitDone()
bank.Rezero()
time.sleep(5.0)
#   #   #   #hand.Hold()
