from arduinoio import serial_control
from protoc.motor_command_pb2 import MotorInitProto
from protoc.motor_command_pb2 import MotorMoveProto
from protoc.motor_command_pb2 import MotorConfigProto
from protoc.motor_command_pb2 import MotorTareProto
from protoc.motor_command_pb2 import MotorMoveAllProto
import math
import logging
from config import ArmConfig
import time
from collections import namedtuple
from threading import Lock

class MotorBank(object):
  def __init__(self, block_mode=False):
    baud = 9600
    device_basename = "ttyACM0"
    self.interface = serial_control.SerialInterface(device_basename, baud=baud)
    device_basename = "ttyACM1"
    self.interface2 = serial_control.SerialInterface(device_basename, baud=baud)
    if block_mode:
        self.AddNewMotors()
    else:
        self.AddMotors()

class Motor(object):
  def __init__(self, interface, gear_factor, hardware_direction=True):
    self.interface = interface
    init_proto = self.InitProto()
    self.address = init_proto.address
    self.SendProto("MINIT", init_proto)
    self.disable_after_moving = True
    self.steps_per_rotation = 200  # for the motor, not the output.
    self.gear_factor = gear_factor
    self.motor_position = 0
    self.done_time = time.time()
    self.hardware_direction = hardware_direction
    self.write_mutex = Lock()
    self.name = ""

  def CurrentPosition(self):
    return self.motor_position / self.StepsPerRadian()

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
    with self.write_mutex:
        move_proto = self.CreateMoveProto()
        move_proto.max_speed = abs(speed) * self.StepsPerSecond()
        move_proto.min_speed = 100
        move_proto.direction = speed > 0.0
        if speed == 0.0:
            move_proto.steps = 0
        else:
            move_proto.steps = self.StepsPerSecond()
        move_proto.use_absolute_steps = False
        if move_proto.direction:
            self.motor_position += move_proto.steps
        else:
            self.motor_position -= move_proto.steps
        if not self.hardware_direction:
            move_proto.direction = not move_proto.direction
        self.done_time = time.time() + 0.75 + move_proto.steps / self.StepsPerSecond()
        return self.Move(move_proto)

  def Stop(self):
      return self.MoveRelative(0.0)

  def MoveAbsolute(self, speed, world_radians):
    return self.Move(self.MoveAbsoluteProto(speed, world_radians))

  def MoveAbsoluteProto(self, speed, world_radians):
    """Speed should range from -1.0 to 1.0"""
    with self.write_mutex:
        move_proto = self.CreateMoveProto()
        move_proto.max_speed = abs(speed) * self.StepsPerSecond()
        move_proto.min_speed = 100
        move_proto.absolute_steps = int(world_radians * self.StepsPerRadian())
        logging.info("%s moving to: %d = %f * %f at %f", self.name, move_proto.absolute_steps, world_radians, self.StepsPerRadian(), move_proto.max_speed)
        #logging.info("Move to: %d at %.02f", move_proto.absolute_steps, move_proto.max_speed)
        move_proto.use_absolute_steps = True
        old_position = self.motor_position
        self.motor_position = move_proto.absolute_steps
        self.done_time = time.time() + 1.0 + 1.0 * abs(self.motor_position - old_position) / self.StepsPerSecond()
        logging.info("Expect done in %0.2f seconds", self.done_time - time.time())
        if not self.hardware_direction:
            move_proto.absolute_steps *= -1
        return move_proto

  def Configure(self, microsteps, max_steps, min_steps, set_zero=False, name=""):
    if name:
      self.name = name
    config_proto = MotorConfigProto()
    config_proto.address = self.address
    for i in range(5):
      steps_test = 1 << i
      if steps_test >= microsteps:
        config_proto.ms0 = i & 0x01
        config_proto.ms1 = i & 0x02
        config_proto.ms2 = i & 0x04
        break
    logging.info("Config ms0, ms1, ms2: %d, %d, %d", config_proto.ms0, config_proto.ms1, config_proto.ms2)
    config_proto.min_steps = min_steps
    config_proto.max_steps = max_steps
    config_proto.zero = set_zero
    self.SendProto("MCONF", config_proto)
    with self.write_mutex:
        self.microsteps = microsteps

  def Tare(self, world_radians):
    logging.info("Taring %s", self.name)
    tare_proto = MotorTareProto()
    tare_proto.address = self.address
    tare_proto.tare_to_steps = int(world_radians * self.StepsPerRadian())
    with self.write_mutex:
        self.motor_position = tare_proto.tare_to_steps
    #start = time.time()
    self.SendProto("MTARE", tare_proto)
    #logging.info("Tare to %d took %f seconds.", tare_proto.tare_to_steps, time.time() - start)

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

  def StepsPerSecond(self):
    return 3000 * self.microsteps

  def AngularRotationPerSecond(self):
    return self.StepsPerSecond() / self.StepsPerRadian()

  def StepsPerRadian(self):
    #return 9000 / math.pi
    return self.gear_factor * self.steps_per_rotation / (2 * math.pi)

  def IsDone(self):
    with self.write_mutex:
        return self.done_time <= time.time()


class TopRightMotor(Motor):
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


class MiddleMotor(Motor):
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

class GripMotor(Motor):
    pass

class LeftGripMotor(GripMotor):
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

class RightGripMotor(GripMotor):
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


class TopLeftMotor(Motor):
  def InitProto(self):
    motor_init = MotorInitProto()
    motor_init.address = 2
    motor_init.enable_pin = 17
    motor_init.dir_pin = 14
    motor_init.step_pin = 15
    motor_init.ms0_pin = 18
    motor_init.ms1_pin = 19
    motor_init.ms2_pin = 16
    return motor_init


class StepperBoardMotorMaps(object):
    def __init__(self):
        MotorPins = namedtuple('MotorPins', ['enable', 'dir', 'step'])
        self.motors = (
                MotorPins(22, 23, 24),
                MotorPins(25, 26, 27),  # Motor #1, if #0 is closest to the arduino hat.
                MotorPins(39, 38, 37),
                MotorPins(36, 35, 34),
                MotorPins(45, 44, 43),
                MotorPins(42, 41, 40),
                MotorPins(28, 29, 30),
                MotorPins(31, 32, 33),
                )

    def ConfigureProto(self, motor_init):
        mp = self.motors[motor_init.address]
        motor_init.enable_pin = mp.enable
        motor_init.dir_pin = mp.dir
        motor_init.step_pin = mp.step
        motor_init.ms0_pin = 0
        motor_init.ms1_pin = 0
        motor_init.ms2_pin = 0

SB_MAP = StepperBoardMotorMaps()

class SBMotor(Motor):
  def __init__(self, interface, address, gear_factor, hardware_direction=True):
      self.address = address
      super(SBMotor, self).__init__(interface, gear_factor, hardware_direction)

  def InitProto(self):
    motor_init = MotorInitProto()
    motor_init.address = self.address
    SB_MAP.ConfigureProto(motor_init)
    return motor_init


class MotorBankBase(MotorBank):
  def AddMotors(self):
    arm_config = ArmConfig()
    self.base_motor = MiddleMotor(self.interface2, arm_config.base_gear_factor, hardware_direction=True)
    self.wrist_motor = TopRightMotor(self.interface2, arm_config.wrist_gear_factor, hardware_direction=False)
    self.lift_motor = TopLeftMotor(self.interface2, arm_config.lift_gear_factor)

    self.wrist_tilt_motor = TopLeftMotor(self.interface, arm_config.tilt_gear_factor)
    self.left_grip = TopRightMotor(self.interface, arm_config.grip_gear_factor)
    self.right_grip = MiddleMotor(self.interface, arm_config.grip_gear_factor)
    self.named_motors = {
            "base": self.base_motor,
            "wrist": self.wrist_motor,
            "lift": self.lift_motor,
            "tilt": self.wrist_tilt_motor,
            "left_grip": self.left_grip,
            "right_grip": self.right_grip,
            }
    self.motors = self.named_motors.values()
    for name, motor in self.named_motors.iteritems():
        motor.Configure(microsteps=1, max_steps=60000, min_steps=-60000, name=name)

  def AddNewMotors(self):
    """Can be used instead of AddMotors for the new large-size stepper controllers."""
    arm_config = ArmConfig()
    self.named_motors = {}
    for i in range(8):
        motor = SBMotor(self.interface, i, arm_config.base_gear_factor, hardware_direction=True)
        self.named_motors["MOTOR_%d" % i] = motor
    self.motors = self.named_motors.values()
    for name, motor in self.named_motors.iteritems():
        motor.Configure(microsteps=1, max_steps=60000, min_steps=-60000, name=name, set_zero=True)

  def Rezero(self):
    self.Release()
    for name, motor in self.named_motors.iteritems():
      if name != "lift":
        motor.MoveAbsolute(0.6, 0.0)
    self.WaitDone()

  def Release(self):
    for motor in self.motors:
      motor.SetDisableAfterMoving(True)

  def WriteMany(self, move_protos, included_motor):
      if len(move_protos) != 3:
          logging.error("Bad call to write many")
      move_all_proto = MotorMoveAllProto()
      # Positions don't actually matter. Addresses are used instead.
      move_all_proto.top_left.CopyFrom(move_protos[0])
      move_all_proto.middle.CopyFrom(move_protos[1])
      move_all_proto.top_right.CopyFrom(move_protos[2])
      included_motor.SendProto("MALL", move_all_proto)

  def WaitDone(self):
      logging.info("Waiting for motors done.")
      waiting = True
      while waiting:
          waiting = False
          for motor in self.motors:
              waiting = waiting or not motor.IsDone()
          time.sleep(0.02)
      logging.info("All motors done.")
