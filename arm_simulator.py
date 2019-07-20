import math
import logging
import numpy
import random
from config import ArmConfig, ArmCurrentSettings, CameraView
import physical_map
import collections

START_PHI = math.pi * 1.2
# CCW is positive for both phi and theta


def NormalizeAngle(angle, start=-math.pi, end=math.pi):
    while angle < start:
        angle += 2 * math.pi
    while angle > end:
        angle -= 2 * math.pi
    return angle

from enum import Enum
class ArmState(Enum):
    HUNT = 1
    APPROACH = 2
    LOWER_ON_BOTTLE = 3
    GRAB_BOTTLE = 4
    PICK_UP_BOTTLE = 5
    DISPENSE = 6
    LOWER_BOTTLE = 7
    REPLACE_BOTTLE = 8

class ArmSimulator(object):
    """Simulator for a robot arm.

    Units are in cm, radians, and seconds.
    """
    def __init__(self):
        self.motor_bank = None
        self.enable_physical_control = False
        self.override_camera = None
        self.debug_string = ""
        self.state = ArmState.HUNT

    def Configure(self, arm_config, theta_init=0, phi_init=START_PHI, rho_init=0):
        self.arm_config = arm_config
        self.r0 = arm_config.r0
        self.r1 = arm_config.r1_camera
        self.theta = theta_init
        self.phi = phi_init
        self.rho = rho_init
        self.UpdatePositions()
        self.d_theta = 0
        self.d_phi = 0
        self.d_rho = 0

    def SendCoordsToMotorBank(self, motor_bank):
        self.motor_bank = motor_bank
        self.enable_physical_control = False
        self.SetDisableAfterMove(False)

    def TogglePhysicalControl(self):
        self.enable_physical_control = not self.enable_physical_control

    def SetTargetAbsolute(self, angle, distance, width, height):
        """Width in cm, not radians.

        Height is relative to arm at rho=0"""
        self.target_angle = angle
        self.target_distance = distance
        self.target_width = width
        self.target_height = height

    def SetThetaSpeed(self, d_theta):
        self.d_theta = d_theta

    def SetPhiSpeed(self, d_phi):
        self.d_phi = d_phi

    def SetRhoSpeed(self, d_rho):
        self.d_rho = d_rho

    def UpdatePositions(self):
        self.r0_horiz = math

    def UpdatePositions(self):
        self.r0_horiz = math.cos(self.rho) * self.r0
        self.r0_vertical = math.sin(self.rho) * self.r0
        self.r0_x = self.r0_horiz * math.cos(self.theta)
        self.r0_y = self.r0_horiz * math.sin(self.theta)
        self.r0_z = self.r0_vertical

        self.r1_x = self.r0_x + self.r1 * math.cos(self.phi + self.theta - math.pi)
        self.r1_y = self.r0_y + self.r1 * math.sin(self.phi + self.theta - math.pi)
        self.r1_x_camera = self.r0_x + self.arm_config.r1_camera * math.cos(self.phi + self.theta - math.pi)
        self.r1_y_camera = self.r0_y + self.arm_config.r1_camera * math.sin(self.phi + self.theta - math.pi)
        if self.enable_physical_control and self.motor_bank is not None:
            base_angle = NormalizeAngle(-self.theta)
            wrist_angle = NormalizeAngle(-self.phi + math.pi)
            #self.motor_bank.wrist_motor.MoveAbsolute(1.0, wrist_angle)
            #self.motor_bank.base_motor.MoveAbsolute(1.0, base_angle)
            self.motor_bank.wrist_motor.MoveAbsolute(abs(self.d_phi), self.target_phi)
            self.motor_bank.base_motor.MoveAbsolute(abs(self.d_theta), self.target_theta)
            self.motor_bank.lift_motor.MoveAbsolute(abs(self.d_rho), self.target_rho)
            self.debug_string += "---PHYSICAL--- Moving base = %.02f, wrist = %.02f" % (
                    base_angle, wrist_angle)

    def TargetPosition(self):
        return (self.target_distance * math.cos(self.target_angle),
                self.target_distance * math.sin(self.target_angle),
                self.target_height)

    def OverrideCamera(self, camera_view):
        self.override_camera = camera_view

    def GetCameraView(self):
        """Computes the view from a camera mounted on the end of the arm (r1).
        
        Returns a tuple of (
            horiz_offset_radians,
            width_radians,
            vertical_offset_radians)
        """
        if self.override_camera:
            return self.override_camera
        tx, ty, tz = self.TargetPosition()
        dx = tx - self.r1_x_camera
        dy = ty - self.r1_y_camera
        absolute_horiz_angle = numpy.arctan2(dy, dx)
        r1_angle = self.phi + self.theta - math.pi
        # Left is positive
        #   logging.info("angle to target from tip = %.02f, angle of r1 = %.02f",
        #           absolute_horiz_angle, r1_angle)
        horiz_offset_radians = NormalizeAngle(absolute_horiz_angle - r1_angle)
        distance = math.sqrt(dx * dx + dy * dy)
        width_radians = self.target_width / distance
        # Down is positive
        #logging.info("Vertical offset: %.02f, dist: %.02f", self.r0_vertical - self.target_height, distance)
        #logging.info("Horiz offset rads: %.02f, width rads: %.02f",
        #       horiz_offset_radians, width_radians)
        vertical_offset_radians = numpy.arcsin(
                (- self.r0_vertical + self.target_height) / distance)
        self.debug_string += "---CAMERA---\nhoriz_offset = %.02f\ndistance = %.02f\nwidth = %.02f\nvertical=%.02f\nvert_offset_rad = %.02f" % (
                horiz_offset_radians, distance, width_radians,
                self.r0_vertical, vertical_offset_radians)
        return CameraView(horiz_offset_radians, width_radians, vertical_offset_radians)


    def Step(self, time):
        self.debug_string = ""
        self.theta += self.d_theta * time
        self.phi += self.d_phi * time
        self.phi = max(math.pi * 0.6, min(math.pi * 1.4, self.phi))
        self.rho = max(0.0, min(math.pi * 0.4, self.rho))
        self.rho += self.d_rho * time
        self.UpdatePositions()
        if self.override_camera:
            self.debug_string += "---REAL CAMERA---horiz: %.02f, width: %.02f, vertical: %.02f" % (
                    self.override_camera)
        self.debug_string += "---ABSOLUTE--- Theta = %.02f, Phi = %.02f, Rho = %.02f" % (
                self.theta, self.phi, self.rho)
        #   logging.info("R0: (%d, %d); R1: (%d, %d)", self.r0_x, self.r0_y,
        #           self.r1_x, self.r1_y)

    def ControlStep(self, time, controller):
        if self.state == ArmState.HUNT:
            pass
        elif self.state == ArmState.APPROACH:
            theta_speed, phi_speed, rho_speed = controller.Update(
                    self.GetCameraView(), self.GetArmCurrentPositions(), time)
            rho_speed = 1.0  # Don't want to move this evenly.
            if self.phi < math.pi:
                theta_speed *= -1.0
            self.SetThetaSpeed(theta_speed * 0.2)
            self.SetPhiSpeed(phi_speed * 0.4)
            self.SetRhoSpeed(rho_speed * 0.1)
            self.target_theta = controller.desired_theta
            self.target_phi = controller.desired_phi
            self.target_rho = self.arm_config.pickup_rho
            self.debug_string += "---CONTROL---\ntheta_speed = %.02f\nphi_speed = %.02f\nrho_speed = %.02f" % (theta_speed, phi_speed, rho_speed)
        elif self.state == ArmState.LOWER_ON_BOTTLE:
            self.target_rho = arm_config.grab_rho
            arm_settings_after_vertical = self.GetArmCurrentPositions()
            arm_settings_after_vertical.rho = self.target_rho
            self.target_theta, self.target_phi = physical_map.EstimateAnglesDesired(
                    self.arm_config, arm_settings_after_vertical, 
                    (self.r1_x, self.r1_y))
            self.SetSpeeds(controller.EvenSpeeds(self.target_theta - self.theta, self.target_phi - self.phi, self.target_rho - self.rho))
            # TODO: push absolutes to physical

    def SetDisableAfterMove(self, disable_after_moving):
        for motor in [self.motor_bank.base_motor, self.motor_bank.wrist_motor]:
            motor.SetDisableAfterMoving(disable_after_moving)

    def Simulate(self, sim_step, control_step, total_time, controller):
        last_control_time = 0
        next_control_time = control_step
        for t in numpy.arange(0, total_time, sim_step):
            self.Step(sim_step)
            if (t > next_control_time):
                self.ControlStep(t - last_control_time)
                next_control_time += control_step
                last_control_time = t

    def GetArmCurrentPositions(self):
        return ArmCurrentSettings(self.theta, self.phi, self.rho)

    def Positions(self, controller):
        tx, ty, tz = self.TargetPosition()
        camera_view = self.GetCameraView()
        arm_current = self.GetArmCurrentPositions()
        est_tx, est_ty = physical_map.EstimateTargetPosition(
                camera_view.width_radians,
                camera_view.horiz_offset_radians,
                self.arm_config,
                arm_current)
        plan_r0_x = self.arm_config.r0 * math.cos(controller.desired_theta)
        plan_r0_y = self.arm_config.r0 * math.sin(controller.desired_theta)
        plan_r1_x = self.arm_config.r1_grab * math.cos(controller.desired_phi + controller.desired_theta - math.pi) + plan_r0_x
        plan_r1_y = self.arm_config.r1_grab * math.sin(controller.desired_phi + controller.desired_theta - math.pi) + plan_r0_y
        return {
            "r0_x": self.r0_x,
            "r0_y": self.r0_y,
            "r0_z": self.r0_z,
            "r1_x": self.r1_x,
            "r1_y": self.r1_y,
            "target_x": tx,
            "target_y": ty,
            "target_z": tz,
            "target_est_x": est_tx,
            "target_est_y": est_ty,
            "plan_r0_x": plan_r0_x,
            "plan_r0_y": plan_r0_y,
            "plan_r1_x": plan_r1_x,
            "plan_r1_y": plan_r1_y,
            "debug_string": self.debug_string,
            }

    def Reset(self):
        self.theta = 0
        self.phi = START_PHI
        self.rho = 0
        self.UpdatePositions()
        self.d_theta = 0
        self.d_phi = 0
        self.d_rho = 0
        angle = random.random() * math.pi + math.pi / 4
        self.SetTargetAbsolute(angle,
            self.target_distance,
            self.target_width,
            self.target_height)

    def Replay(self):
        self.theta = 0
        self.phi = START_PHI
        self.rho = 0
        self.UpdatePositions()
        self.d_theta = 0
        self.d_phi = 0
        self.d_rho = 0
