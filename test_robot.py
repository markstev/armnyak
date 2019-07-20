#!/usr/bin/python

from motor2 import MotorBankBase
from io_read import InputBoard
from config import ArmConfig
from protoc.motor_command_pb2 import MotorMoveProto
from hand import Hand
import time
import logging
import math

bank = MotorBankBase()
io_reader = InputBoard()
arm_config = ArmConfig

hand = Hand(bank, io_reader, arm_config)
#hand.Grab()
hand.Hold()
time.sleep(2.0)
hand.Dispense()
time.sleep(10.0)

bank.Release()
bank.Rezero()
#hand.Hold()
