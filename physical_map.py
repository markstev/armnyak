import math
from config import ArmConfig, ArmCurrentSettings

def WidthToDistance(width_radians):
    # sin(width_radians) = bottle_width / distance
    kBottleWidth = 10  # cm
    return kBottleWidth / math.sin(width_radians)

def EstimateTargetPosition(width_radians, horiz_offset_radians, arm_config, arm_current_settings):
    d = WidthToDistance(width_radians)
    r0 = arm_config.r0
    r1 = arm_config.r1
    theta = arm_current_settings.theta
    phi = arm_current_settings.phi
    rho = arm_current_settings.rho
    phi_vs_horizon = phi - math.pi + theta
    tx = r0 * math.cos(theta) * math.cos(rho) + r1 * math.cos(phi_vs_horizon) + d * math.cos(phi_vs_horizon + horiz_offset_radians)
    ty = r0 * math.sin(theta) * math.cos(rho) + r1 * math.sin(phi_vs_horizon) + d * math.sin(phi_vs_horizon + horiz_offset_radians)
    return (tx, ty)

def EstimateAnglesDesired(width_radians, horiz_offset_radians, arm_config, arm_current_settings, target_position):
    tx, ty = target_position
    r0 = arm_config.r0
    r1 = arm_config.r1
    t = math.sqrt(tx * tx + ty * ty)
    t1 = (r1 * r1 - r0 * r0 + t * t) / 2 / t
    t2 = t - t1
    r0_t_angle_target = math.acos(t2 / r0)
    r1_t_angle_target = math.acos(t1 / r1)
    phi_target = 2 * math.pi - (math.pi - r0_t_angle_target - r1_t_angle_target)
    theta_target = numpy.atan2(tx, ty) - r0_t_angle_target
