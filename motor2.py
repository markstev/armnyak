from arduinoio import serial_control
from protoc.motor_command_pb2 import MotorInitProto
from protoc.motor_command_pb2 import MotorMoveProto

class MotorBank(object):
  def __init__(self):
    baud = 9600
    device_basename = "ttyACM"
    self.interface = serial_control.SerialInterface(device_basename, baud=baud)
    self.AddMotors()

class Motor(object):
  def __init__(self, interface):
    self.interface = interface
    self.SendProto("MINIT", self.InitProto())

  def Move(self, motor_move_proto):
    self.SendProto("MUP", motor_move_proto)

  def SendProto(self, name, proto):
    serialized = proto.SerializeToString()
    raw_message = []
    for x in name:
      raw_message.append(x)
    for x in serialized:
      raw_message.append(x)
    #command = name + "".join(raw_message)
    command = raw_message
    print "Command: %s" % command
    print "Command: %s" % [ord(x) for x in raw_message]
    print "Command length: %d" % len(command)
    self.interface.Write(0, command)

class BaseMotor(Motor):
 def InitProto(self):
    motor_init = MotorInitProto()
    motor_init.address = 0
    motor_init.enable_pin = 13
    motor_init.dir_pin = 8
    motor_init.step_pin = 9
    motor_init.ms0_pin = 12
    motor_init.ms1_pin = 11
    motor_init.ms2_pin = 10
    return motor_init

class MotorBankBase(MotorBank):
  def AddMotors(self):
    self.base_motor = BaseMotor(self.interface)
