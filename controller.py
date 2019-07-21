import math
import physical_map
import config
import logging


def EvenSpeeds(arm_config, delta_theta, delta_phi, delta_rho):
    angle_and_gear_factors = [(delta_theta, arm_config.base_gear_factor),
            (delta_phi, arm_config.wrist_gear_factor),
            (delta_rho, arm_config.lift_gear_factor)]
    times = []
    for angle, gear in angle_and_gear_factors:
        time_to_move = abs(angle) * gear
        times.append(time_to_move)
        logging.info("Time: %f", time_to_move)
    total_time = max(times)
    def sign(x):
        if x >= 0:
            return 1
        else:
            return -1
    def speed(angle_change, time_to_reach_angle, total_time_to_move):
        if total_time_to_move == 0:
            return 0.0
        return sign(angle_change) * min(1.0, time_to_reach_angle / total_time_to_move)
    return (speed(delta_theta, times[0], total_time),
            speed(delta_phi, times[1], total_time),
            speed(delta_rho, times[2], total_time))

class MotorController(object):
    def __init__(self, m, b):
        self.m = m
        self.b = b
        self.history = []

    def GetNextSpeed(self, target_offset, time_delta, other_contributor):
        """
        Returns an angular speed to drive a motor at.

        target_offset and output speed are basically unknown units. Let's see if we can figure them out on the fly.
        
        Assume we're outputting rads/sec
        """
        output = (target_offset - self.b) / self.m
        chosen_output = max(-1.0, min(1.0, output))
        self.history.append((target_offset, time_delta, other_contributor, chosen_output))
        # TODO: regressions
        return chosen_output


class Controller(object):
    def __init__(self):
        self.base_motor = MotorController(0.2, 0)
        self.lift_motor = MotorController(0.1, 0)
        self.wrist_motor = MotorController(0.4, 0)
        self.desired_theta = 0
        self.desired_phi = 0

    def Update(self, camera_view, arm_current_settings, time_delta):
        """
        Base target is distance to target, inferred from width_radians
        Lift target is vertical_offset_radians -> 0
        Wrist target is horiz_offset_radians -> 0
        """
        rho = self.lift_motor.GetNextSpeed(camera_view.vertical_offset_radians, time_delta, 0)
        target_width_radians = 0.7  # 5cm width at 10cm away
        theta = self.base_motor.GetNextSpeed(target_width_radians - camera_view.width_radians, time_delta, -rho)
        phi = self.wrist_motor.GetNextSpeed(camera_view.horiz_offset_radians, time_delta, theta)
        return (theta, phi, rho)


class MathController(object):
    def __init__(self, arm_config):
        self.base_motor = MotorController(0.2, 0)
        self.lift_motor = MotorController(0.1, 0)
        self.wrist_motor = MotorController(0.4, 0)
        self.arm_config = arm_config
        self.desired_theta = 0
        self.desired_phi = 0

    def Update(self, camera_view, arm_current, time_delta):
        """
        Base target is distance to target, inferred from width_radians
        Lift target is vertical_offset_radians -> 0
        Wrist target is horiz_offset_radians -> 0
        """
        rho = self.lift_motor.GetNextSpeed(camera_view.vertical_offset_radians, time_delta, 0)
        target_position = physical_map.EstimateTargetPosition(camera_view.width_radians,
                camera_view.horiz_offset_radians, self.arm_config, arm_current)
        desired_theta, desired_phi = physical_map.EstimateAnglesDesired(
                self.arm_config, arm_current, target_position)
        self.desired_theta = desired_theta
        self.desired_phi = desired_phi
        delta_theta = desired_theta - arm_current.theta
        delta_phi = desired_phi - arm_current.phi
        return EvenSpeeds(self.arm_config, delta_theta, delta_phi, 0.0)
