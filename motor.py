#!/usr/bin/python
from arduinoio import serial_control
import struct

class Motor(object):
  def __init__(self):
    baud = 9600
    device_basename = "ttyACM"
    self.interface = serial_control.SerialInterface(device_basename, baud=baud)

  def Move(self, stepper_dir_pin, stepper_pulse_pin, negative_trigger_pin,
             positive_trigger_pin, done_pin, forward, steps, final_wait,
             max_wait, temp_pin, temp_pin_threshold):
    raw_message = []
    if forward:
      forward = 0x01
    else:
      forward = 0x00
    raw_message.extend((stepper_dir_pin, stepper_pulse_pin,
                        negative_trigger_pin, positive_trigger_pin, done_pin,
                        forward))
    if max_wait < 1000:
      raw_message.append(10)
    else:
      raw_message.append(0)
    raw_message.append(temp_pin)
    raw_message.extend(struct.unpack('4B', struct.pack('<i', steps)))
    raw_message.extend(struct.unpack('4B', struct.pack('<i', temp_pin_threshold)))
    # print "max_wait: %s" % max_wait
    # raw_message.extend(struct.unpack('4B', struct.pack('<i', 4000)))
    raw_message = [chr(x) for x in raw_message]
    command = "MOVE" + "".join(raw_message)
    print "Move command: %s" % [ord(x) for x in raw_message]
    self.interface.Write(0, command)
    while self.interface.GetSlaveState() == serial_control.SlaveState.UNKNOWN:
      pass

