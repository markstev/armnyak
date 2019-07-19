import math

class Hand(object):
    def __init__(self, motor_bank, io_reader, arm_config):
        self.motor_bank = motor_bank
        self.io_reader = io_reader
        self.arm_config = arm_config
        self.grab_amount = math.pi * 2 * 10
        self.motor_bank.wrist_tilt_motor.SetDisableAfterMoving(False)

    def Calibrate(self):
        pass

    def Dispense(self):
        self.motor_bank.wrist_tilt_motor.MoveAbsolute(1.0, -math.pi * 0.75)
        time.sleep(5)
        self.motor_bank.wrist_tilt_motor.MoveAbsolute(1.0, -math.pi * 0.1)
        time.sleep(2)
        self.motor_bank.wrist_tilt_motor.MoveAbsolute(0.2, math.pi * 0.1)
        self.io_reader.RegisterCallback(self.arm_config.wrist_tilt_pin, False,
                lamda: self.motor_bank.wrist_tilt_motor.Tare(0.0))
        # TODO: What if we don't get to the hall effect?

    def Grab(self):
        self.motor_bank.left_grip.SetDisableAfterMoving(False)
        self.motor_bank.right_grip.SetDisableAfterMoving(False)
        self.motor_bank.left_grip.MoveAbsolute(1.0, self.grab_amount)
        self.motor_bank.left_grip.MoveAbsolute(1.0, -self.grab_amount)

    def Release(self):
        self.motor_bank.left_grip.SetDisableAfterMoving(True)
        self.motor_bank.right_grip.SetDisableAfterMoving(True)
        self.motor_bank.left_grip.MoveAbsolute(1.0, -self.grab_amount)
        self.motor_bank.left_grip.MoveAbsolute(1.0, self.grab_amount)
        time.sleep(5)
        self.RecalibrateGrabber()

    def RecalibrateGrabber(self):
        # Slowly extend until we see the white section.
