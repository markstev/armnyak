import math
from config import ArmConfig, ArmCurrentSettings
import numpy

def WidthToDistance(width_radians):
    # sin(width_radians) = bottle_width / distance
    kBottleWidth = 10  # cm
    return kBottleWidth / math.sin(width_radians)

def EstimateTargetPosition(width_radians, horiz_offset_radians, arm_config, arm_current_settings):
    d = WidthToDistance(width_radians)
    r0_pivot = arm_config.r0_pivot
    r0_flat = arm_config.r0_flat
    r1 = arm_config.r1_camera
    theta = arm_current_settings.theta
    phi = arm_current_settings.phi
    rho = arm_current_settings.rho
    phi_vs_horizon = phi - math.pi + theta
    tx = r0_pivot * math.cos(theta) * math.cos(rho) + r0_flat * math.cos(theta) + r1 * math.cos(phi_vs_horizon) + d * math.cos(phi_vs_horizon + horiz_offset_radians)
    ty = r0_pivot * math.sin(theta) * math.cos(rho) + r0_flat * math.sin(theta) + r1 * math.sin(phi_vs_horizon) + d * math.sin(phi_vs_horizon + horiz_offset_radians)
    return (tx, ty)

def EstimateAnglesDesired(arm_config, arm_current_settings, target_position):
    tx, ty = target_position
    r0 = arm_config.r0_pivot * math.cos(arm_current_settings.rho) + arm_config.r0_flat
    r1 = arm_config.r1_grab
    t = math.sqrt(tx * tx + ty * ty)
    max_arm_grab_length = arm_config.r1_grab + r0
    if t > max_arm_grab_length:
        # If this is true, the equaiton is not solvable, probably due to a bad target estimate. Let's assume it's closer than we estimated and solve anyway.
        old_t = t
        t = max_arm_grab_length * .95
        tx *= t / old_t
        ty *= t / old_t
    t1 = (r1 * r1 - r0 * r0 + t * t) / 2 / t
    t2 = t - t1
    r0_t_angle_target = math.acos(t2 / r0)
    r1_t_angle_target = math.acos(t1 / r1)
    phi_target = 2 * math.pi - (math.pi - r0_t_angle_target - r1_t_angle_target)
    theta_target = numpy.arctan2(ty, tx) - r0_t_angle_target
    return (theta_target, phi_target)


def GetGrabPosition(arm_config, arm_current_settings):
    theta = arm_current_settings.theta
    phi = arm_current_settings.phi
    rho = arm_current_settings.rho
    grab_x = (arm_config.r0_pivot * math.cos(theta) * math.cos(rho) +
            arm_config.r0_flat * math.cos(theta) +
            arm_config.r1_grab * math.cos(phi - math.pi + theta))
    grab_y = (arm_config.r0_pivot * math.sin(theta) * math.cos(rho) +
            arm_config.r0_flat * math.sin(theta) +
            arm_config.r1_grab * math.sin(phi - math.pi + theta))
    return (grab_x, grab_y)
