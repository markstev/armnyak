import math
import time

class Hand(object):
    def __init__(self, motor_bank, io_reader, arm_config):
        self.motor_bank = motor_bank
        self.io_reader = io_reader
        self.arm_config = arm_config
        self.grab_amount = math.pi * 2 * 3
        self.motor_bank.wrist_tilt_motor.SetDisableAfterMoving(False)

    def Calibrate(self):
        pass

    def Dispense(self):
        self.motor_bank.wrist_tilt_motor.MoveAbsolute(0.6, -math.pi * 0.7)
        time.sleep(1.75)
        self.motor_bank.wrist_tilt_motor.MoveAbsolute(0.6, -math.pi * 0.1)
        time.sleep(1)
        self.motor_bank.wrist_tilt_motor.MoveAbsolute(0.2, math.pi * 0.05)
        #   self.io_reader.RegisterCallback(self.arm_config.wrist_tilt_pin, False,
        #           lamda: self.motor_bank.wrist_tilt_motor.Tare(0.0))
        # TODO: What if we don't get to the hall effect?

    def Hold(self):
        """Use after you've already grabbed. Probably only needed for test/dev
        programs."""
        self.motor_bank.left_grip.SetDisableAfterMoving(False)
        self.motor_bank.right_grip.SetDisableAfterMoving(False)
        self.motor_bank.left_grip.MoveAbsolute(1.0, -0.5 * math.pi)
        self.motor_bank.right_grip.MoveAbsolute(1.0, 0.5 * math.pi)

    def Grab(self):
        self.motor_bank.left_grip.SetDisableAfterMoving(False)
        self.motor_bank.right_grip.SetDisableAfterMoving(False)
        self.motor_bank.left_grip.MoveAbsolute(1.0, -self.grab_amount)
        self.motor_bank.right_grip.MoveAbsolute(1.0, self.grab_amount)
        time.sleep(1.0)

    def Release(self):
        self.motor_bank.left_grip.SetDisableAfterMoving(True)
        self.motor_bank.right_grip.SetDisableAfterMoving(True)
        self.motor_bank.left_grip.MoveAbsolute(1.0, self.grab_amount)
        self.motor_bank.right_grip.MoveAbsolute(1.0, -self.grab_amount)
        time.sleep(5)
        self.RecalibrateGrabber()

    def RecalibrateGrabber(self):
        # Slowly extend until we see the white section.
        pass
