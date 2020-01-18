from arduinoio import serial_control
from protoc.io_read_pb2 import IOReadProto
from protoc.io_read_pb2 import ConfigureIOReadProto
import logging
import threading
import time
import collections
from threading import Lock

PinCallback = collections.namedtuple('PinCallback', 'pin trigger_value callback permanent')

# PIN 22 = wrist tilt hall effect
# PIN 24 = wrist hall effect
# PIN 26 = bottle bumper switch
# PIN 28 = one optical sensor
# PIN 30 = other optical sensor
# PIN 7 = SONAR trigger
# PIN 6 = SONAR echo

class Reader(object):
    def __init__(self):
        self.Reconnect()

    def Reconnect(self):
        self.contiguous_errors = 0
        baud = 9600
        device_basename = "ttyACM1"
        self.interface = serial_control.SerialInterface(device_basename, baud=baud)

    def Read(self):
        if self.contiguous_errors > 5:
            logging.info("Reconnecting due to errors.")
            self.Reconnect()
        message = self.interface.Read(no_checksums=True, expected_length=-1, verbose=False)
        if message:
            read_proto = IOReadProto()
            #logging.info("".join([chr(x) for x in message.command]))
            command_str = "".join([chr(x) for x in message.command])
            try:
              read_proto.ParseFromString(command_str)
              self.contiguous_errors = 0
              #logging.info("Good parse %s", message.command)
            except:
              logging.info("Parse proto error %s", message.command)
              self.contiguous_errors += 1
              return False
            #logging.info(read_proto.enable_read_bits_0)
            return read_proto
        return False

    def Configure(self):
        config_proto = ConfigureIOReadProto()
        config_proto.enable_read_bits_0 = 30
        config_proto.enable_read_bits_1 = 23
        self.SendProto("READ", config_proto)
        logging.info("Sent configure.")

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
        self.callbacks = {}
        self.reads = 0
        self.write_mutex = Lock()

    def Start(self):
        self.thread.start()
        self.start_time = time.time()

    def ReadsPerSec(self):
        return self.reads * 1.0 / (time.time() - self.start_time)

    def GetPin(self, pin_number):
        return self.GetPinInternal(pin_number, self.pin_values)

    def GetPinInternal(self, pin_number, pin_values):
        #logging.info("mask = %x", (0x01 << (pin_number % 32)))
        #logging.info("masked = %x", (0x01 << (pin_number % 32)) & self.pin_values[pin_number / 32])
        return pin_values[pin_number / 32] & (0x01 << (pin_number % 32)) != 0x00

    def UpdateRead(self):
        while True:
            message = self.reader.Read()
            if message:
                self.reads += 1
                last_pin_values = list(self.pin_values)
                self.pin_values[0] = message.read_bits_0
                self.pin_values[1] = message.read_bits_1
                if message.dist_cm < 500:
                   if message.dist_cm != 0.0:
                       #logging.info("DIST CM = %.02f   ENCODER = %d", message.dist_cm, -42)#message.encoder_0_count)
                       pass
                callbacks_to_delete = []
                with self.write_mutex:
                    for pin, pin_callback in self.callbacks.iteritems():
                        pin_value = self.GetPin(pin) 
                        if (pin_value == pin_callback.trigger_value and
                                # Only trigger on change.
                                pin_value != self.GetPinInternal(pin, last_pin_values)):
                            pin_callback.callback()
                            if not pin_callback.permanent:
                                callbacks_to_delete.append(pin)
                            #logging.info("TRIGGER: %d from %d to %d", pin, self.GetPinInternal(pin, last_pin_values), pin_value)
                        #else:
                        #   logging.info("Not triggering: %d from %d to %d", pin, self.GetPinInternal(pin, last_pin_values), pin_value)
                    for cb in callbacks_to_delete:
                        del self.callbacks[cb]

    def RegisterCallback(self, pin, trigger_value, callback, permanent=False):
        with self.write_mutex:
            self.callbacks[pin] = PinCallback(pin, trigger_value, callback, permanent)


#   read_proto = IOReadProto()
#   read_proto.enable_read_bits_0 = 41
#   read_proto.enable_read_bits_1 = 42
#   read_proto.read_bits_0 = 43
#   read_proto.read_bits_1 = 44
#   read_proto.dist_cm =42.42
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
