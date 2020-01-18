import math
import time
import logging

class Hand(object):
    def __init__(self, motor_bank, io_reader, arm_config):
        self.motor_bank = motor_bank
        self.io_reader = io_reader
        self.arm_config = arm_config
        self.grab_amount = math.pi * 2 * 6
        self.motor_bank.wrist_tilt_motor.SetDisableAfterMoving(False)

    def Calibrate(self):
        self.motor_bank.wrist_tilt_motor.MoveAbsolute(0.5, -math.pi * 1.5)
        def StopWrist():
            logging.info("Stop wrist.")
            self.motor_bank.wrist_tilt_motor.Stop()
            self.motor_bank.wrist_tilt_motor.Tare(-math.pi * 1.1)
            self.motor_bank.wrist_tilt_motor.MoveAbsolute(1.0, 0.0)
        self.io_reader.RegisterCallback(self.arm_config.wrist_tilt_hall_pin, False,
                StopWrist, permanent=False)

    def Dispense(self):
        self.motor_bank.wrist_motor.MoveAbsolute(0.4, -math.pi / 3.0)
        time.sleep(0.75)
        self.motor_bank.wrist_tilt_motor.MoveAbsolute(0.6, -math.pi * 0.7)
        time.sleep(2.50)
        self.motor_bank.wrist_tilt_motor.MoveAbsolute(0.6, -math.pi * 0.0)
        #time.sleep(1)
        #self.motor_bank.wrist_tilt_motor.MoveAbsolute(0.2, math.pi * 0.05)
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

    def Rotate(self, angle):
        self.motor_bank.wrist_motor.MoveAbsolute(1.0, angle)
    def Grab(self):
        self.motor_bank.left_grip.SetDisableAfterMoving(False)
        self.motor_bank.right_grip.SetDisableAfterMoving(False)
        self.motor_bank.left_grip.MoveAbsolute(0.2, -self.grab_amount)
        self.motor_bank.right_grip.MoveAbsolute(0.2, self.grab_amount)
        def StopGrab():
            logging.info("Stop grab.")
            # Force stop
            #self.motor_bank.left_grip.Configure(microsteps=1, max_steps=0, min_steps=0)
            #self.motor_bank.right_grip.Configure(microsteps=1, max_steps=0, min_steps=0)
            # Set position.
            self.motor_bank.left_grip.Stop()
            self.motor_bank.right_grip.Stop()
            #self.motor_bank.left_grip.MoveAbsolute(1.0, -self.grab_amount / 2)
            #self.motor_bank.right_grip.MoveAbsolute(1.0, self.grab_amount / 2)
            #self.motor_bank.left_grip.Configure(microsteps=1, max_steps=6000, min_steps=-6000)
            #self.motor_bank.right_grip.Configure(microsteps=1, max_steps=6000, min_steps=-6000)

        self.io_reader.RegisterCallback(self.arm_config.bottle_bumper_pin, False,
                StopGrab)
        time.sleep(1.0)

    def Release(self):
        self.motor_bank.left_grip.Tare(0.0)
        self.motor_bank.right_grip.Tare(0.0)
        self.motor_bank.left_grip.SetDisableAfterMoving(True)
        self.motor_bank.right_grip.SetDisableAfterMoving(True)
        self.motor_bank.left_grip.MoveAbsolute(1.0, self.grab_amount * 0.75)
        self.motor_bank.right_grip.MoveAbsolute(1.0, -self.grab_amount * 0.75)
        self.RecalibrateGrabber()

    def RecalibrateGrabber(self):
        # Slowly extend until we see the white section.
        pass
