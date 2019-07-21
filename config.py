import collections
import math

# Units are cm, radians, and seconds

class ArmConfig(object):
    def __init__(self):
        self.r0_flat = 6 * 2.54
        self.r0_pivot = 12 * 2.54
        self.r0 = self.r0_flat + self.r0_pivot
        self.r1_camera = 1 * 2.54
        # Distance from pivot to grab area.
        self.r1_grab = 5.5 * 2.54

        self.target_width = 3 * 2.54

        self.base_gear_factor = 39.27
        self.lift_gear_factor = 80 * 2
        self.wrist_gear_factor = 7 * 2

        self.tilt_gear_factor = 7 * 2
        self.grip_gear_factor = 1

        self.pickup_rho = math.pi / 3
        self.grab_rho = 0.349066
        self.dispense_rho = math.pi / 2
        self.bottle_bumper_pin = 26
        self.lift_switch_pin = 32

    def ApplyTares(self, motor_bank, input_board):
        #AddTare(MOTOR, PIN, POSITION)
        Tare = collections.namedtuple('Tare', 'motor pin trigger_value world_radians')
        tares = [Tare(motor_bank.wrist_tilt_motor, 22, False, -math.pi),
                ]
        for tare in tares:
            input_board.RegisterCallback(tare.pin, tare.trigger_value,
                    lambda: tare.motor.Tare(tare.world_radians),
                    permanent=True)

CameraView = collections.namedtuple('CameraView', 'horiz_offset_radians width_radians vertical_offset_radians')

ArmCurrentSettings = collections.namedtuple('ArmCurrentSettings', 'theta phi rho')


# Sonar Pins:
# From front (sonar pinging you) vcc, trigger, echo, ground
