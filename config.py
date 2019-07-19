import collections

# Units are cm, radians, and seconds

class ArmConfig(object):
    def __init__(self):
        self.r0 = 12 * 2.54
        self.r1_camera = 10 * 2.54
        self.r1_grab = 13 * 2.54

        self.target_width = 10

        self.base_gear_factor = 39.27
        self.lift_gear_factor = 80
        self.wrist_gear_factor = 7

        self.tilt_gear_factor = 7
        self.grip_gear_factor = 1

CameraView = collections.namedtuple('CameraView', 'horiz_offset_radians width_radians vertical_offset_radians')

ArmCurrentSettings = collections.namedtuple('ArmCurrentSettings', 'theta phi rho')


# Sonar Pins:
# From front (sonar pinging you) vcc, trigger, echo, ground
