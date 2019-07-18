from arduinoio import serial_control
from protoc.io_read_pb2 import IOReadProto
from protoc.io_read_pb2 import ConfigureIOReadProto
import logging
import threading
import time

class Reader(object):
    def __init__(self):
        baud = 9600
        device_basename = "ttyACM1"
        self.interface = serial_control.SerialInterface(device_basename, baud=baud)

    def Read(self):
        message = self.interface.Read(no_checksums=True, expected_length=-1, verbose=False)
        if message:
            read_proto = IOReadProto()
            #logging.info("".join([chr(x) for x in message.command]))
            try:
              read_proto.ParseFromString("".join([chr(x) for x in message.command]))
            except:
              logging.info("Parse proto error")
            #logging.info(read_proto.enable_read_bits_0)
            return read_proto
        return False

    def Configure(self):
        config_proto = ConfigureIOReadProto()
        config_proto.enable_read_bits_0 = 30
        config_proto.enable_read_bits_1 = 23
        self.SendProto("READ", config_proto)

    def SendProto(self, name, proto):
      serialized = proto.SerializeToString()
      raw_message = []
      for x in name:
        raw_message.append(x)
      for x in serialized:
        raw_message.append(x)
      command = raw_message
      self.interface.Write(0, command)


class InputBoard(object):
    def __init__(self):
        self.reader = Reader()
        self.thread = threading.Thread(name="Read", target=self.UpdateRead)
        self.thread.daemon = True
        self.pin_values = [0, 0]

    def Start(self):
        self.thread.start()

    def GetPin(self, pin_number):
        #logging.info("mask = %x", (0x01 << (pin_number % 32)))
        #logging.info("masked = %x", (0x01 << (pin_number % 32)) & self.pin_values[pin_number / 32])
        return self.pin_values[pin_number / 32] & (0x01 << (pin_number % 32)) != 0x00

    def UpdateRead(self):
        while True:
            message = self.reader.Read()
            if message:
                self.pin_values[0] = message.read_bits_0
                self.pin_values[1] = message.read_bits_1
                #if message.dist_cm < 500:
                logging.info("DIST CM = %.02f", message.dist_cm)


#   read_proto = IOReadProto()
#   read_proto.enable_read_bits_0 = 41
#   read_proto.enable_read_bits_1 = 42
#   read_proto.read_bits_0 = 43
#   read_proto.read_bits_1 = 44
#   logging.info("Message should be: %s", read_proto.SerializeToString())
board = InputBoard()
board.Start()
while True:
    logging.info("P24=%d P25=%d P26=%d", board.GetPin(24), board.GetPin(25), board.GetPin(26))
    time.sleep(0.2)
#   logging.info("Start loop")
#   reader.Configure()
#   logging.info("Start Read")
#   while True:
#       if reader.Read():
#           break
