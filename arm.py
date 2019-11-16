import math
import time
import logging
from controller import EvenSpeeds
from config import ArmCurrentSettings
import physical_map

class Arm(object):
    def __init__(self, motor_bank, io_reader, arm_config):
        self.motor_bank = motor_bank
        self.io_reader = io_reader
        self.arm_config = arm_config

    def RaiseFully(self):
        self.motor_bank.lift_motor.MoveAbsolute(0.8, math.pi / 2)

    def Calibrate(self):
        if self.io_reader.GetPin(self.arm_config.lift_switch_pin) == False:
            raise_amount = math.pi / 8
            self.motor_bank.lift_motor.MoveAbsolute(0.8, raise_amount)
            time.sleep(2.0)
        logging.info("Lower now")
        self.Lower()

    def SetupLowerStop(self):
        def StopLower():
            logging.info("Stop lower.")
            self.motor_bank.lift_motor.Stop()
            self.motor_bank.lift_motor.Tare(self.arm_config.grab_rho)
            self.motor_bank.lift_motor.MoveAbsolute(1.0, self.arm_config.grab_rho)
            limit_position = self.motor_bank.lift_motor.CurrentPosition()
            #logging.info("Maybe set min to %d", limit_position)
            #self.motor_bank.lift_motor.Configure(microsteps=1, max_steps=60000, min_steps=limit_position)
        self.io_reader.RegisterCallback(self.arm_config.lift_switch_pin, False,
                StopLower, permanent=True)

    def Lower(self):
        # Negative is down.
        #lower_amount = -math.pi / 2 + self.arm_config.grab_rho
        #lower_amount = math.pi / 2 - self.arm_config.grab_rho
        if not self.io_reader.GetPin(self.arm_config.lift_switch_pin):
            return
        lower_amount = -math.pi / 2
        self.motor_bank.lift_motor.MoveAbsolute(0.8, lower_amount)
        self.SetupLowerStop()

    def GoToReleasePositionHeight(self):
        self.motor_bank.lift_motor.MoveAbsolute(0.8, self.arm_config.release_rho)

    def GoToPositionHeight(self):
        self.motor_bank.lift_motor.MoveAbsolute(0.8, self.arm_config.pickup_rho)

    def Rotate(self, angle):
        self.motor_bank.base_motor.MoveAbsolute(0.6, angle)

    def LowerOntoBottle(self):
        arm_settings = ArmCurrentSettings(theta=0.0, phi=3.141592653589793, rho=0.7853981633974483)

        #   arm_settings = ArmCurrentSettings(
        #       self.motor_bank.base_motor.CurrentPosition(),
        #       math.pi + self.motor_bank.wrist_motor.CurrentPosition(),
        #       self.motor_bank.lift_motor.CurrentPosition())
        arm_settings_after_vertical = ArmCurrentSettings(arm_settings.theta,
                arm_settings.phi, self.arm_config.grab_rho)
        logging.info("Arm settings = %s", arm_settings)
        target_theta, target_phi = physical_map.EstimateAnglesDesired(
                self.arm_config, arm_settings_after_vertical, 
                physical_map.GetGrabPosition(self.arm_config, arm_settings))
        #target_theta *= 1.4
        logging.info("Targets: %f, %f, %f", target_theta, target_phi - math.pi,
                arm_settings_after_vertical.rho)
        speeds = EvenSpeeds(self.arm_config, target_theta - arm_settings.theta,
                target_phi - arm_settings.phi,
                arm_settings_after_vertical.rho - arm_settings.rho)
        logging.info("Speeds: %s", speeds)
        #self.motor_bank.lift_motor.MoveAbsolute(abs(speeds[2]), (arm_settings_after_vertical.rho + self.arm_config.pickup_rho * 3 ) / 4)
        #self.motor_bank.WaitDone()
        self.motor_bank.lift_motor.MoveAbsolute(abs(speeds[2]), arm_settings_after_vertical.rho)
        self.motor_bank.base_motor.MoveAbsolute(abs(speeds[0]), target_theta)
        self.motor_bank.wrist_motor.MoveAbsolute(abs(speeds[1]), target_phi - math.pi)
        self.motor_bank.lift_motor.MoveAbsolute(abs(speeds[2]), arm_settings_after_vertical.rho)
        self.lower_speeds = speeds
        self.lowered_theta = target_theta
        self.lowered_phi = target_phi - math.pi
        self.raised_theta = arm_settings.theta
        self.raised_phi = arm_settings.phi - math.pi

    def SetWristToGrabPosition(self):
        self.motor_bank.wrist_motor.MoveAbsolute(0.4, self.lowered_phi)

    def RaiseOverDroppedBottle(self):
        logging.info("Release with speeds %s", self.lower_speeds)
        self.motor_bank.base_motor.MoveAbsolute(abs(self.lower_speeds[0]),
                self.raised_theta)
        self.motor_bank.wrist_motor.MoveAbsolute(abs(self.lower_speeds[1]),
                self.raised_phi)
        self.motor_bank.lift_motor.MoveAbsolute(abs(self.lower_speeds[2]),
                self.arm_config.pickup_rho)
