import math
import logging
import numpy
import random
from config import ArmConfig, ArmCurrentSettings

START_PHI = math.pi * 1.2
# CCW is positive for both phi and theta

def NormalizeAngle(angle, start=-math.pi, end=math.pi):
    while angle < start:
        angle += 2 * math.pi
    while angle > end:
        angle -= 2 * math.pi
    return angle

class ArmSimulator(object):
    """Simulator for a robot arm.

    Units are in cm, radians, and seconds.
    """
    def __init__(self):
        self.motor_bank = None
        self.enable_physical_control = False
        self.override_camera = None
        self.debug_string = ""

    def Configure(self, r0, r1, theta_init=0, phi_init=START_PHI, rho_init=0):
        self.r0 = r0
        self.r1 = r1
        self.theta = theta_init
        self.phi = phi_init
        self.rho = rho_init
        self.UpdatePositions()
        self.d_theta = 0
        self.d_phi = 0
        self.d_rho = 0

    def SendCoordsToMotorBank(self, motor_bank):
        self.motor_bank = motor_bank
        self.enable_physical_control = True
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
        if self.enable_physical_control and self.motor_bank is not None:
            base_angle = NormalizeAngle(-self.theta)
            wrist_angle = NormalizeAngle(-self.phi + math.pi)
            self.motor_bank.wrist_motor.MoveAbsolute(1.0, wrist_angle)
            self.motor_bank.base_motor.MoveAbsolute(1.0, base_angle)
            self.debug_string += "---PHYSICAL--- Moving base = %.02f, wrist = %.02f" % (
                    base_angle, wrist_angle)

    def TargetPosition(self):
        return (self.target_distance * math.cos(self.target_angle),
                self.target_distance * math.sin(self.target_angle),
                self.target_height)

    def OverrideCamera(self, horiz_offset_radians, width_radians, vertical_offset_radians):
        self.override_camera = (
                horiz_offset_radians,
                width_radians,
                vertical_offset_radians)

    def CameraView(self):
        """Computes the view from a camera mounted on the end of the arm (r1).
        
        Returns a tuple of (
            horiz_offset_radians,
            width_radians,
            vertical_offset_radians)
        """
        if self.override_camera:
            return self.override_camera
        tx, ty, tz = self.TargetPosition()
        dx = tx - self.r1_x
        dy = ty - self.r1_y
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
        return (horiz_offset_radians, width_radians, vertical_offset_radians)


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
        theta_speed, phi_speed, rho_speed = controller.Update(
                self.CameraView(), time)
        if self.phi < math.pi:
            theta_speed *= -1.0
        #   logging.info(
        #           "Controller update speeds: (theta, phi, rho): (%.02f, %.02f, %.02f)",
        #           theta_speed, phi_speed, rho_speed)
        self.debug_string += "---CONTROL---\ntheta_speed = %.02f\nphi_speed = %.02f\nrho_speed = %.02f" % (theta_speed, phi_speed, rho_speed)
        self.SetThetaSpeed(theta_speed * 0.2)
        self.SetPhiSpeed(phi_speed * 0.4)
        self.SetRhoSpeed(rho_speed * 0.1)

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

    def Positions(self):
        tx, ty, tz = self.TargetPosition()
        return {
            "r0_x": self.r0_x,
            "r0_y": self.r0_y,
            "r0_z": self.r0_z,
            "r1_x": self.r1_x,
            "r1_y": self.r1_y,
            "target_x": tx,
            "target_y": ty,
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
