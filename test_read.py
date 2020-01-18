from arduinoio import serial_control
from protoc.io_read_pb2 import IOReadProto
from protoc.io_read_pb2 import ConfigureIOReadProto
import io_read
import logging
import threading
import time

read_proto = IOReadProto()
read_proto.enable_read_bits_0 = 41
read_proto.enable_read_bits_1 = 42
read_proto.read_bits_0 = 43
read_proto.read_bits_1 = 44
read_proto.dist_cm = 0
#read_proto.encoder_0_count = 0
logging.info("Message should be: %s", read_proto.SerializeToString())
board = io_read.InputBoard()
board.Start()
while True:
    s = "\n------\n"
    for i in range(22, 34, 2):
        s += " PIN %d = %d\n" % (i, board.GetPin(i))

    print(s)
    logging.info("RPS = %f %s", board.ReadsPerSec(), s)
    time.sleep(0.2)
#   logging.info("Start loop")
#   reader.Configure()
#   logging.info("Start Read")
#   while True:
#       if reader.Read():
#           break
