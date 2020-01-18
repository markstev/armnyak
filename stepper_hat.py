from arduinoio import serial_control
from protoc.io_read_pb2 import IOReadProto
from protoc.lcd_pb2 import WriteLCDProto
from protoc.io_read_pb2 import ConfigureIOReadProto
from motor2 import MotorBankBase
from protoc.motor_command_pb2 import MotorMoveProto
import logging
import threading
import time
import collections
import math
from threading import Lock

class StepperHat(object):
    def __init__(self):
        self.Reconnect()

    def Reconnect(self):
        self.contiguous_errors = 0
        baud = 9600
        device_basename = "ttyACM0"
        self.interface = serial_control.SerialInterface(device_basename, baud=baud)

    def SendProto(self, name, proto):
      serialized = proto.SerializeToString()
      raw_message = []
      for x in name:
        raw_message.append(x)
      for x in serialized:
        raw_message.append(x)
      command = raw_message
      self.interface.Write(0, command)

logging.info("Connect!")
hat = StepperHat()

def WriteLCD(printout):
    lcd_proto = WriteLCDProto()
    lcd_proto.cursor_x = 0
    lcd_proto.cursor_y = 0
    lcd_proto.printout = printout
    hat.SendProto("LCDP", lcd_proto)

logging.info("Connected!")
for i in range(1):
    lcd_proto = WriteLCDProto()
    lcd_proto.cursor_x = 0
    lcd_proto.cursor_y = 0
    lcd_proto.printout = "lololololololol! %d" % i
    logging.info("Send command.")
    hat.SendProto("LCDP", lcd_proto)
    logging.info("Sent command.")
    time.sleep(1)

bank = MotorBankBase(block_mode=True)
forward = 1.0
for i in range(2):
  forward *= -1.0
  for name, motor in sorted(bank.named_motors.iteritems()):
      if name != "left_grip":
          continue
      # #WriteLCD(name)
      # #motor.MoveAbsolute(1.0, 0)
      # #time.sleep(1)
      # continue
      logging.info("Run motor: %s", name)
      motor.MoveAbsolute(1.0, forward * 0.1 * math.pi)
      time.sleep(3.0)
      #WriteLCD(name)
    #logging.info("P24=%d P25=%d P26=%d", input_board.GetPin(24), input_board.GetPin(25), input_board.GetPin(26))
for name, motor in sorted(bank.named_motors.iteritems()):
  motor.MoveAbsolute(1.0, 0.0)
time.sleep(20)

#   read_proto = IOReadProto()
#   read_proto.enable_read_bits_0 = 41
#   read_proto.enable_read_bits_1 = 42
#   read_proto.read_bits_0 = 43
#   read_proto.read_bits_1 = 44
#   logging.info("Message should be: %s", read_proto.SerializeToString())
#   board = InputBoard()
#   board.Start()
#   while True:
#       logging.info("P24=%d P25=%d P26=%d", board.GetPin(24), board.GetPin(25), board.GetPin(26))
#       time.sleep(0.2)
#   logging.info("Start loop")
#   reader.Configure()
#   logging.info("Start Read")
#   while True:
#       if reader.Read():
#           break
