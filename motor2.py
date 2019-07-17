from arduinoio import serial_control
from protoc.motor_command_pb2 import MotorInitProto
from protoc.motor_command_pb2 import MotorMoveProto
from protoc.motor_command_pb2 import MotorConfigProto
import math
import logging

class MotorBank(object):
  def __init__(self):
    baud = 9600
    device_basename = "ttyACM"
    self.interface = serial_control.SerialInterface(device_basename, baud=baud)
    self.AddMotors()
    self.microsteps = 1

class Motor(object):
  def __init__(self, interface):
    self.interface = interface
    init_proto = self.InitProto()
    self.address = init_proto.address
    self.SendProto("MINIT", init_proto)
    self.disable_after_moving = True

  def Move(self, motor_move_proto):
    self.SendProto("MUP", motor_move_proto)

  def SetDisableAfterMoving(self, disable_after_moving):
    self.disable_after_moving = disable_after_moving

  def CreateMoveProto(self):
    move_proto = MotorMoveProto()
    move_proto.address = self.address
    move_proto.disable_after_moving = self.disable_after_moving
    #Should override one of these sections
    move_proto.direction = True
    move_proto.max_speed = 0.1
    move_proto.min_speed = 0.0
    move_proto.steps = 1
    move_proto.use_absolute_steps = False
    move_proto.absolute_steps = 0
    return move_proto

  def MoveRelative(self, speed):
    """Speed should range from -1.0 to 1.0"""
    move_proto = self.CreateMoveProto()
    move_proto.max_speed = abs(speed) * 1500 * self.microsteps
    move_proto.min_speed = 100
    move_proto.direction = speed > 0.0
    move_proto.steps = self.StepsPerSecond()
    move_proto.use_absolute_steps = False
    return self.Move(move_proto)

  def MoveAbsolute(self, speed, world_radians):
    """Speed should range from -1.0 to 1.0"""
    move_proto = self.CreateMoveProto()
    move_proto.max_speed = abs(speed) * 1500 * self.microsteps
    move_proto.min_speed = 100
    move_proto.absolute_steps = int(world_radians * self.StepsPerRadian())
    #logging.info("Move to: %d at %.02f", move_proto.absolute_steps, move_proto.max_speed)
    move_proto.use_absolute_steps = True
    return self.Move(move_proto)

  def Configure(self, microsteps, max_steps, min_steps, set_zero=False):
    config_proto = MotorConfigProto()
    config_proto.address = self.address
    for i in range(5):
      steps_test = 1 << i
      if steps_test >= microsteps:
        config_proto.ms0 = i & 0x01
        config_proto.ms1 = i & 0x02
        config_proto.ms2 = i & 0x04
        break
    config_proto.min_steps = min_steps
    config_proto.max_steps = max_steps
    config_proto.zero = set_zero
    self.SendProto("MCONF", config_proto)
    self.microsteps = microsteps

  def SendProto(self, name, proto):
    serialized = proto.SerializeToString()
    raw_message = []
    for x in name:
      raw_message.append(x)
    for x in serialized:
      raw_message.append(x)
    #command = name + "".join(raw_message)
    command = raw_message
    #   print "Command: %s" % command
    #   print "Command: %s" % [ord(x) for x in raw_message]
    #   print "Command length: %d" % len(command)
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

  def StepsPerSecond(self):
    return 20000 / 6

  def AngularRotationPerSecond(self):
    return math.pi / 3.0

  def StepsPerRadian(self):
    return 9000 / math.pi


class WristMotor(Motor):
  def InitProto(self):
    motor_init = MotorInitProto()
    motor_init.address = 1
    motor_init.enable_pin = 7
    motor_init.dir_pin = 3
    motor_init.step_pin = 2
    motor_init.ms0_pin = 6
    motor_init.ms1_pin = 5
    motor_init.ms2_pin = 4
    return motor_init

  def StepsPerSecond(self):
    return 2600

  def AngularRotationPerSecond(self):
    return math.pi

  def StepsPerRadian(self):
    return 2600 / math.pi


class MotorBankBase(MotorBank):
  def AddMotors(self):
    self.base_motor = BaseMotor(self.interface)
    self.wrist_motor = WristMotor(self.interface)
